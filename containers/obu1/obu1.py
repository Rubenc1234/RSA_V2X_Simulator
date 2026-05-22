"""OBU1 Agent - Descentralizado (Cenário Arnacó Braga)."""

import json
import time
import threading
import sys
import signal
import os
from types import SimpleNamespace
import math

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("ERROR: paho-mqtt not installed. Run: pip install paho-mqtt")
    sys.exit(1)

try:
    import simulator_core as core
    from scenarios.registry import get_scenario_config, get_vehicle_config
    from simulator_core import (
        CLEAR_DISTANCE_M,
        DENM_CAUSE_COLLISION_RISK,
        REVERSE_DISTANCE_M,
        YIELD_DISTANCE_M,
        WARNING_DISTANCE_M,
        TICK_SECONDS,
        VehicleSim,
        choose_yield_vehicle,
        vehicles_are_approaching_each_other,
        vehicles_share_same_road_and_opposite_direction,
        vehicle_nearest_exit,
        haversine_meters,
        bearing_degrees,
    )
except ImportError as e:
    print(f"ERROR: Cannot import simulator_core: {e}")
    print("Make sure simulator_core.py is in /app or project root")
    sys.exit(1)

RUNNING = True
DEFAULT_PROFILE = {
    "name": "obu1",
    "stationId": 2,
    "broker": "localhost",
    "startPoint": (41.727849, -8.163264),
    "endPoint": (41.725639, -8.165512),
    "baseSpeedMps": 8.0,
}


def signal_handler(_signum, _frame):
    global RUNNING
    RUNNING = False
    print("\n[OBU1] Shutting down...")


class OBU1Agent:
    """OBU1 Agent - Autónomo, publica CAM e detecta colisões."""

    AGENT_NAME = "obu1"

    def __init__(self):
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.peer_clients = []

        self.scenario = get_scenario_config()
        self.profile = self.load_profile()
        self.station_id = int(self.profile["stationId"])
        self.broker = self.profile.get("broker", os.environ.get("MQTT_BROKER", "localhost"))
        self.port = int(os.environ.get("MQTT_PORT", "1883"))
        self.vehicle = VehicleSim(
            name=self.profile["name"],
            station_id=self.station_id,
            broker_host=self.broker,
            start_point=tuple(self.profile["startPoint"]),
            end_point=tuple(self.profile["endPoint"]),
            base_speed_mps=float(self.profile["baseSpeedMps"]),
            loop_route=bool(self.profile.get("loopRoute", True)),
        )

        # Dados de outros OBUs recebidos via MQTT (chave: stationId)
        self.other_vehicle_data: dict = {}
        
        # Controle de evitamento
        self.last_denm_time = 0.0
        self.avoidance_active = False
        self.yield_mode = "stop"
        self.yield_resume_time = 0.0
        self.yield_vehicle_name = ""
        self.denm_hold_until = 0.0

        self.peer_brokers = self._resolve_peer_brokers()
        for index, peer_broker in enumerate(self.peer_brokers, start=1):
            self._start_peer_listener(peer_broker, f"{self.AGENT_NAME}-peer-{index}")

        self.start_time = time.time()
        self.incident_after_seconds = self.profile.get("incidentAfterSeconds")
        self.incident_cause_code = int(self.profile.get("incidentCauseCode", 2))
        self.incident_sub_cause_code = int(self.profile.get("incidentSubCauseCode", 0))
        self.incident_validity_duration = int(self.profile.get("incidentValidityDuration", 20))
        self.emit_collision_risk_denm = bool(self.profile.get("emitCollisionRiskDenm", True))
        self.incident_triggered = False
        self.last_published_event_pos = None

    def load_profile(self) -> dict:
        """Load the vehicle profile for this agent from the active scenario."""
        return get_vehicle_config(self.AGENT_NAME, default=DEFAULT_PROFILE)

    def _resolve_peer_brokers(self) -> list[str]:
        """Return the MQTT brokers for the other stations in the active scenario."""
        scenario_brokers = self.scenario.get("brokers", [])
        peer_hosts = [
            broker.get("host")
            for broker in scenario_brokers
            if broker.get("host") and broker.get("host") != self.broker
        ]
        if peer_hosts:
            return peer_hosts

        fallback_peer = os.environ.get("MQTT_PEER_BROKER")
        return [fallback_peer] if fallback_peer else []

    def _start_peer_listener(self, broker_host, client_suffix):
        client = mqtt.Client(client_id=client_suffix)
        client.on_message = self.on_message
        try:
            client.connect(broker_host, self.port, keepalive=60)
        except Exception as e:
            print(f"[OBU1] Erro ao ligar ao broker peer {broker_host}:{self.port}: {e}")
            return
        client.subscribe("vanetza/out/cam")
        client.subscribe("vanetza/out/denm")
        client.loop_start()
        self.peer_clients.append(client)

    def on_connect(self, client, userdata, flags, rc):
        """Callback: quando conectado ao MQTT."""
        print(f"[OBU1] Conectado ao MQTT broker (rc={rc})")
        self.mqtt_client.subscribe("vanetza/out/cam")
        self.mqtt_client.subscribe("vanetza/out/denm")

    def on_message(self, client, userdata, msg):
        """Callback: quando recebe mensagem MQTT."""
        try:
            payload = json.loads(msg.payload.decode())

            # Vanetza wraps decoded data under fields.header / fields.cam
            station_id = payload.get("fields", {}).get("header", {}).get("stationId")

            if "cam" in msg.topic or payload.get("fields", {}).get("cam") is not None:
                # Ignore our own CAMs and malformed ones
                if station_id is None or station_id == self.station_id:
                    return
                candidate_cam = payload.get("fields", {}).get("cam")
                pos = self.extract_position_from_cam(candidate_cam)
                if self.is_valid_position(pos):
                    # store by station id so we can handle many peers generically
                    self.other_vehicle_data[station_id] = candidate_cam
                    print(f"[OBU1] Recebi CAM de station_id={station_id}")

            elif "denm" in msg.topic or payload.get("fields", {}).get("denm") is not None:
                print(f"[OBU1]  Recebi DENM: {payload.get('fields', {}).get('denm', {}).get('management', {}).get('eventPosition')}")
                self.apply_denm_reaction(payload)
        except Exception as e:
            print(f"[OBU1] Erro ao processar mensagem: {e}")

    def is_valid_position(self, pos):
        """Reject placeholder and malformed CAM coordinates."""
        if not pos:
            return False
        lat, lon = pos
        if lat is None or lon is None:
            return False
        if lat == 40.0 and lon == -8.0:
            return False
        return -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0

    def apply_denm_reaction(self, payload):
        """Ativa a reação de cedência quando um DENM chega."""
        peer_vehicle = self.build_peer_vehicle_view()
        if not peer_vehicle:
            return

        # Soft-yield: both vehicles reduce speed (no full stop), pass slowly, then resume.
        yield_vehicle = choose_yield_vehicle(self.vehicle, peer_vehicle)
        self.yield_vehicle_name = yield_vehicle.name
        self.avoidance_active = True
        self.last_denm_time = time.time()

        # Determine event distance (best-effort)
        denm = payload.get("fields", {}).get("denm", {})
        event_position = denm.get("management", {}).get("eventPosition", {})
        event_lat = event_position.get("latitude")
        event_lon = event_position.get("longitude")
        if event_lat is not None and event_lon is not None:
            event_distance = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, event_lat, event_lon)
        else:
            event_distance = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, peer_vehicle.current_lat, peer_vehicle.current_lon)

        # Soft yield parameters
        self.yield_mode = "soft"
        self.denm_hold_until = time.time() + 6.0

        # Both vehicles slow: yielding vehicle slows more, priority vehicle slows less.
        slow_factor_yield = 0.35
        slow_factor_other = 0.6

        if self.vehicle.name == self.yield_vehicle_name:
            new_speed = max(0.5, self.vehicle.base_speed_mps * slow_factor_yield)
            self.vehicle.target_speed_mps = new_speed
        else:
            # If already very close, reduce a bit more
            if event_distance <= YIELD_DISTANCE_M:
                new_speed = max(0.6, self.vehicle.base_speed_mps * (slow_factor_other - 0.15))
            else:
                new_speed = max(0.8, self.vehicle.base_speed_mps * slow_factor_other)
            self.vehicle.target_speed_mps = new_speed

        print(
            f"[OBU1] DENM recebido -> {self.yield_vehicle_name} cede ({self.yield_mode}); "
            f"evento_dist={event_distance:.1f}m alvo={self.vehicle.target_speed_mps:.1f}m/s"
        )

    def extract_position_from_cam(self, cam_payload):
        """Extrai (lat, lon) do CAM decodificado."""
        try:
            basic = cam_payload["camParameters"]["basicContainer"]["referencePosition"]
            return (basic["latitude"], basic["longitude"])
        except Exception:
            return None

    def extract_heading_from_cam(self, cam_payload):
        """Extrai heading do CAM decodificado."""
        try:
            high = cam_payload["camParameters"]["highFrequencyContainer"][
                "basicVehicleContainerHighFrequency"
            ]
            return high["heading"]["headingValue"]
        except Exception:
            return 0.0

    def build_peer_vehicle_view(self):
        """Cria uma vista mínima do OBU2 para reutilizar a lógica de prioridade."""
        # Choose the most relevant peer vehicle from known CAMs. Strategy:
        # - ignore ourself
        # - pick the nearest vehicle by haversine distance
        if not self.other_vehicle_data:
            return None

        best = None
        best_dist = float("inf")
        for sid, cam in self.other_vehicle_data.items():
            pos = self.extract_position_from_cam(cam)
            if not pos:
                continue
            lat, lon = pos
            dist = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, lat, lon)
            if dist < best_dist:
                best_dist = dist
                best = (sid, cam)

        if not best:
            return None

        sid, peer_cam = best
        lat, lon = self.extract_position_from_cam(peer_cam)
        heading = self.extract_heading_from_cam(peer_cam)

        # Estimate a short route for the peer based on its heading so functions
        # like `choose_yield_vehicle` (which expect start/end points and route
        # distance helpers) work without hardcoded station ids.
        def _destination_point(lat0, lon0, bearing_deg, distance_m=300):
            R = 6371000.0
            br = math.radians(bearing_deg)
            lat1 = math.radians(lat0)
            lon1 = math.radians(lon0)
            d = distance_m
            lat2 = math.asin(math.sin(lat1) * math.cos(d / R) + math.cos(lat1) * math.sin(d / R) * math.cos(br))
            lon2 = lon1 + math.atan2(
                math.sin(br) * math.sin(d / R) * math.cos(lat1),
                math.cos(d / R) - math.sin(lat1) * math.sin(lat2),
            )
            return (math.degrees(lat2), math.degrees(lon2))

        est_start = _destination_point(lat, lon, (heading + 180) % 360, distance_m=300)
        est_end = _destination_point(lat, lon, heading, distance_m=300)

        return SimpleNamespace(
            name=f"station_{sid}",
            station_id=sid,
            current_lat=lat,
            current_lon=lon,
            start_point=est_start,
            end_point=est_end,
            last_heading_deg=heading,
            distance_from_route_start_m=lambda: haversine_meters(est_start[0], est_start[1], lat, lon),
            distance_to_route_end_m=lambda: haversine_meters(lat, lon, est_end[0], est_end[1]),
        )

    def publish_cam_loop(self):
        """OBU1 publica seu próprio CAM a 5 Hz."""
        print("[OBU1] Iniciando publicação de CAM (5 Hz)")
        iteration = 0
        while RUNNING:
            try:
                if (
                    not self.incident_triggered
                    and self.incident_after_seconds is not None
                    and (time.time() - self.start_time) >= float(self.incident_after_seconds)
                ):
                    self.incident_triggered = True
                    self.vehicle.target_speed_mps = 0.0
                    self.vehicle.current_speed_mps = 0.0
                    print(f"[OBU1] INCIDENTE SIMULADO -> veículo parado e DENM de acidente emitido")
                    core.publish_denm(
                        self.vehicle,
                        cause_code=self.incident_cause_code,
                        sub_cause_code=self.incident_sub_cause_code,
                        validity_duration=self.incident_validity_duration,
                        event_position=(self.vehicle.current_lat, self.vehicle.current_lon),
                    )

                self.vehicle.step_and_publish(TICK_SECONDS)
                # If vehicle reached final waypoint, optionally trigger incident
                if not self.incident_triggered and self.vehicle.segment_idx >= len(self.vehicle.route) - 1:
                    self.incident_triggered = True
                    self.vehicle.target_speed_mps = 0.0
                    self.vehicle.current_speed_mps = 0.0
                    print(f"[OBU1] INCIDENTE (on arrival) -> veículo parado e DENM de acidente emitido")
                    core.publish_denm(
                        self.vehicle,
                        cause_code=self.incident_cause_code,
                        sub_cause_code=self.incident_sub_cause_code,
                        validity_duration=self.incident_validity_duration,
                        event_position=(self.vehicle.current_lat, self.vehicle.current_lon),
                    )
                iteration += 1

                if iteration % 5 == 0:  # Log a cada 1 segundo
                    print(
                        f"[OBU1] CAM: lat={self.vehicle.current_lat:.6f} "
                        f"lon={self.vehicle.current_lon:.6f} "
                        f"speed={self.vehicle.current_speed_mps:.1f}m/s"
                    )

                time.sleep(TICK_SECONDS)
            except Exception as e:
                print(f"[OBU1] Erro ao publicar CAM: {e}")
                time.sleep(TICK_SECONDS)

    def collision_detection_loop(self):
        """OBU1 verifica colisões autonomamente."""
        print("[OBU1] Iniciando detecção de colisão")
        while RUNNING:
            try:
                peer_vehicle = self.build_peer_vehicle_view()
                if peer_vehicle:
                    my_lat = self.vehicle.current_lat
                    my_lon = self.vehicle.current_lon
                    peer_lat = peer_vehicle.current_lat
                    peer_lon = peer_vehicle.current_lon

                    distance = haversine_meters(my_lat, my_lon, peer_lat, peer_lon)
                    bearing_to_peer = bearing_degrees(my_lat, my_lon, peer_lat, peer_lon)
                    same_road_opposite = vehicles_share_same_road_and_opposite_direction(self.vehicle, peer_vehicle)
                    approaching_each_other = vehicles_are_approaching_each_other(self.vehicle, peer_vehicle)
                    close_range_risk = distance <= YIELD_DISTANCE_M

                    if int(time.time()) % 2 == 0:
                        print(f"[OBU1] Distância para OBU2: {distance:.1f}m (bearing: {bearing_to_peer:.1f}°)")

                    now = time.time()
                    # Require approach vector for both opposite-road and close-range
                    # detections to avoid alerts when vehicles follow the same lane
                    # at similar speed (overtake/spacing noise).
                    risk_window = (
                        (same_road_opposite and approaching_each_other)
                        or (close_range_risk and approaching_each_other)
                    )
                    if risk_window and distance < WARNING_DISTANCE_M:
                        if not self.avoidance_active and now - self.last_denm_time > 5.0:
                            # Global policy: only one vehicle publishes DENM per encounter.
                            # Deterministic leader election by lower station_id.
                            is_global_denm_emitter = self.station_id < peer_vehicle.station_id
                            yield_vehicle = choose_yield_vehicle(self.vehicle, peer_vehicle)
                            self.yield_vehicle_name = yield_vehicle.name
                            self.yield_mode = "stop"
                            self.yield_resume_time = 0.0
                            self.avoidance_active = True
                            self.denm_hold_until = now + 4.0

                            yield_side = vehicle_nearest_exit(yield_vehicle)
                            yield_distance = (
                                yield_vehicle.distance_from_route_start_m()
                                if yield_side == "start"
                                else yield_vehicle.distance_to_route_end_m()
                            )

                            print(f"\n[OBU1] AVISO DE COLISÃO")
                            print(f"       Distância: {distance:.1f}m < {WARNING_DISTANCE_M}m")
                            print(f"       {self.yield_vehicle_name} deve ceder ({self.yield_mode})")
                            print(f"       Margem livre na rota: {yield_distance:.1f}m")

                            if is_global_denm_emitter:
                                # Mirror DENM to both brokers (local + peer) so observers
                                # always see the alert regardless of who originated it.
                                notify = [SimpleNamespace(client=self.vehicle.client)]
                                if self.peer_clients:
                                    notify.append(SimpleNamespace(client=self.peer_clients[0]))

                                event_position = (
                                    (my_lat + peer_lat) / 2.0,
                                    (my_lon + peer_lon) / 2.0,
                                )

                                # Deduplicate quick re-publishes: if the last published
                                # event is very close in space and recent in time, skip.
                                publish_ok = True
                                if self.last_published_event_pos is not None and self.last_denm_time > 0.0:
                                    last_lat, last_lon = self.last_published_event_pos
                                    moved = haversine_meters(last_lat, last_lon, event_position[0], event_position[1])
                                    if moved < 15.0 and (now - self.last_denm_time) < 20.0:
                                        publish_ok = False

                                if publish_ok:
                                    core.publish_denm(
                                        self.vehicle,
                                        cause_code=DENM_CAUSE_COLLISION_RISK,
                                        validity_duration=10,
                                        event_position=event_position,
                                        notify_vehicles=notify,
                                    )
                                    self.last_published_event_pos = event_position
                                else:
                                    print(f"[OBU1] Skipping duplicate DENM (moved={moved:.1f}m recent={now - self.last_denm_time:.1f}s)")
                            else:
                                print(f"[OBU1] Risco detetado; aguardar DENM do station_{peer_vehicle.station_id}")

                            # Increase hold time to avoid frequent re-emission
                            self.last_denm_time = now
                            self.denm_hold_until = now + 12.0

                    if self.avoidance_active:
                        # Soft-yield speed targets
                        slow_factor_yield = 0.35
                        slow_factor_other = 0.6

                        if self.vehicle.name == self.yield_vehicle_name:
                            # Yielding vehicle slows more
                            self.vehicle.target_speed_mps = max(0.5, self.vehicle.base_speed_mps * slow_factor_yield)
                        else:
                            # Priority vehicle slows less; if very close, reduce a bit more
                            if distance <= YIELD_DISTANCE_M:
                                self.vehicle.target_speed_mps = max(0.6, self.vehicle.base_speed_mps * (slow_factor_other - 0.15))
                            else:
                                self.vehicle.target_speed_mps = max(0.8, self.vehicle.base_speed_mps * slow_factor_other)

                        # Clear avoidance when vehicles passed or hold timeout expired
                        if (
                            (distance > CLEAR_DISTANCE_M and not approaching_each_other)
                            or now >= self.denm_hold_until
                        ):
                            print("[OBU1] Perigo passou, retomando velocidade normal\n")
                            self.avoidance_active = False
                            self.yield_vehicle_name = ""
                            self.yield_mode = "soft"
                            # small resume delay to avoid oscillation
                            self.yield_resume_time = now + 1.0
                            self.denm_hold_until = 0.0

                    if not self.avoidance_active:
                        self.vehicle.target_speed_mps = self.vehicle.base_speed_mps

                time.sleep(0.5)
            except Exception as e:
                print(f"[OBU1] Erro em detecção colisão: {e}")
                time.sleep(0.5)

    def denm_listener_loop(self):
        """OBU1 subscreve DENMs e reage."""
        print("[OBU1] Iniciando listener de DENM")
        first = True
        while RUNNING:
            try:
                if not first:
                    time.sleep(1.0)
                first = False
            except Exception as e:
                print(f"[OBU1] Erro em DENM listener: {e}")
                time.sleep(1.0)

    def run(self):
        """Inicia o agente OBU1."""
        print("\n" + "=" * 60)
        print(f" {self.profile['name'].upper()} AGENTE DESCENTRALIZADO - {self.scenario.get('label', 'cenário')}")
        print("=" * 60)
        print(f" Station ID: {self.station_id}")
        print(f" Rota: {self.vehicle.start_point} → {self.vehicle.end_point}")
        print(f" Broker: {self.broker}:{self.port}")
        if self.peer_brokers:
            print(f" Peers: {', '.join(self.peer_brokers)}")
        print("=" * 60 + "\n")

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        try:
            # Conectar ao MQTT (Vanetza container)
            self.mqtt_client.connect(self.broker, self.port, keepalive=60)
            print("[OBU1] Conectado ao broker MQTT\n")
        except Exception as e:
            print(f"[OBU1] Erro ao conectar MQTT: {e}")
            sys.exit(1)

        # Threads autónomos
        threading.Thread(target=self.publish_cam_loop, daemon=True).start()
        if self.emit_collision_risk_denm:
            threading.Thread(target=self.collision_detection_loop, daemon=True).start()
        else:
            print("[OBU1] Collision DENM disabled by scenario profile")
        threading.Thread(target=self.denm_listener_loop, daemon=True).start()

        # Loop MQTT (bloqueia até SIGINT)
        try:
            self.mqtt_client.loop_forever()
        except KeyboardInterrupt:
            pass
        finally:
            print("\n[OBU1] Encerrando...")
            self.vehicle.close()
            self.mqtt_client.disconnect()
            for client in self.peer_clients:
                client.loop_stop()
                client.disconnect()


if __name__ == "__main__":
    obu1 = OBU1Agent()
    obu1.run()
