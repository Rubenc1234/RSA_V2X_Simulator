"""OBU1 Agent - Descentralizado (Cenário Arnacó Braga)."""

import json
import time
import threading
import sys
import signal
import os
from types import SimpleNamespace

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("ERROR: paho-mqtt not installed. Run: pip install paho-mqtt")
    sys.exit(1)

try:
    import simulator_core as core
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


def signal_handler(_signum, _frame):
    global RUNNING
    RUNNING = False
    print("\n[OBU1] Shutting down...")


class OBU1Agent:
    """OBU1 Agent - Autónomo, publica CAM e detecta colisões."""

    def __init__(self):
        self.station_id = 2
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.peer_clients = []

        # OBU1: Rua Arnaçó, Braga (direção 1)
        self.broker = os.environ.get("MQTT_BROKER", "localhost")
        self.port = int(os.environ.get("MQTT_PORT", "1883"))
        self.peer_broker = os.environ.get("MQTT_PEER_BROKER", "obu2")
        self.vehicle = VehicleSim(
            name="obu1",
            station_id=2,
            broker_host=self.broker,
            start_point=(41.727849, -8.163264),
            end_point=(41.725762, -8.165449),
            base_speed_mps=8.0,
            loop_route=True,
        )

        # Dados recebidos de OBU2 via MQTT
        self.other_vehicle_data = {}
        self.other_station_id = 3  # OBU2
        
        # Controle de evitamento
        self.last_denm_time = 0.0
        self.avoidance_active = False
        self.yield_mode = "stop"
        self.yield_resume_time = 0.0
        self.yield_vehicle_name = ""

        self.peer_start_point = (41.725639, -8.165512)
        self.peer_end_point = (41.727598, -8.163408)

        self._start_peer_listener(self.peer_broker, "obu1-peer")

    def _start_peer_listener(self, broker_host, client_suffix):
        client = mqtt.Client(client_id=client_suffix)
        client.on_message = self.on_message
        client.connect(broker_host, self.port, keepalive=60)
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
                if station_id == self.other_station_id:
                    # store the decoded cam object (fields.cam) for position extraction
                    self.other_vehicle_data = payload.get("fields", {}).get("cam")
                    print(f"[OBU1] Recebi CAM de OBU2 (station_id={station_id})")

            elif "denm" in msg.topic or payload.get("fields", {}).get("denm") is not None:
                print(f"[OBU1]  Recebi DENM: {payload.get('fields', {}).get('denm', {}).get('management', {}).get('eventPosition')}")
                self.apply_denm_reaction(payload)
        except Exception as e:
            print(f"[OBU1] Erro ao processar mensagem: {e}")

    def apply_denm_reaction(self, payload):
        """Ativa a reação de cedência quando um DENM chega."""
        peer_vehicle = self.build_peer_vehicle_view()
        if not peer_vehicle:
            return

        same_road_opposite = vehicles_share_same_road_and_opposite_direction(self.vehicle, peer_vehicle)
        approaching_each_other = vehicles_are_approaching_each_other(self.vehicle, peer_vehicle)
        if not (same_road_opposite and approaching_each_other):
            return

        yield_vehicle = choose_yield_vehicle(self.vehicle, peer_vehicle)
        self.yield_vehicle_name = yield_vehicle.name
        self.avoidance_active = True
        self.last_denm_time = time.time()

        yield_side = vehicle_nearest_exit(yield_vehicle)
        yield_distance = (
            yield_vehicle.distance_from_route_start_m()
            if yield_side == "start"
            else yield_vehicle.distance_to_route_end_m()
        )

        denm = payload.get("fields", {}).get("denm", {})
        event_position = denm.get("management", {}).get("eventPosition", {})
        event_lat = event_position.get("latitude")
        event_lon = event_position.get("longitude")
        if event_lat is not None and event_lon is not None:
            event_distance = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, event_lat, event_lon)
        else:
            event_distance = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, peer_vehicle.current_lat, peer_vehicle.current_lon)

        self.yield_mode = "stop"
        if self.vehicle.name == self.yield_vehicle_name:
            if yield_side == "start" and yield_distance <= REVERSE_DISTANCE_M:
                self.yield_mode = "reverse"
                self.vehicle.target_speed_mps = -min(2.5, max(1.2, self.vehicle.base_speed_mps * 0.35))
            else:
                self.vehicle.target_speed_mps = 0.0 if event_distance <= YIELD_DISTANCE_M else self.vehicle.base_speed_mps * 0.25
        else:
            self.vehicle.target_speed_mps = self.vehicle.base_speed_mps * (0.80 if event_distance > YIELD_DISTANCE_M else 0.60)

        print(
            f"[OBU1] DENM recebido -> {self.yield_vehicle_name} cede ({self.yield_mode}); "
            f"margem={yield_distance:.1f}m"
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
        if not self.other_vehicle_data:
            return None

        peer_cam = self.other_vehicle_data
        pos = self.extract_position_from_cam(peer_cam)
        if not pos:
            return None

        heading = self.extract_heading_from_cam(peer_cam)
        lat, lon = pos

        return SimpleNamespace(
            name="obu2",
            station_id=self.other_station_id,
            current_lat=lat,
            current_lon=lon,
            start_point=self.peer_start_point,
            end_point=self.peer_end_point,
            last_heading_deg=heading,
            distance_from_route_start_m=lambda: haversine_meters(*self.peer_start_point, lat, lon),
            distance_to_route_end_m=lambda: haversine_meters(lat, lon, *self.peer_end_point),
        )

    def publish_cam_loop(self):
        """OBU1 publica seu próprio CAM a 5 Hz."""
        print("[OBU1] Iniciando publicação de CAM (5 Hz)")
        iteration = 0
        while RUNNING:
            try:
                self.vehicle.step_and_publish(TICK_SECONDS)
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

                    if int(time.time()) % 2 == 0:
                        print(f"[OBU1] Distância para OBU2: {distance:.1f}m (bearing: {bearing_to_peer:.1f}°)")

                    now = time.time()
                    if same_road_opposite and approaching_each_other and distance < WARNING_DISTANCE_M:
                        if not self.avoidance_active and now - self.last_denm_time > 5.0:
                            yield_vehicle = choose_yield_vehicle(self.vehicle, peer_vehicle)
                            self.yield_vehicle_name = yield_vehicle.name
                            self.yield_mode = "stop"
                            self.yield_resume_time = 0.0
                            self.avoidance_active = True

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

                            notify = None
                            if self.peer_clients:
                                notify = [SimpleNamespace(client=self.peer_clients[0])]

                            event_position = (
                                (my_lat + peer_lat) / 2.0,
                                (my_lon + peer_lon) / 2.0,
                            )

                            core.publish_denm(
                                self.vehicle,
                                cause_code=DENM_CAUSE_COLLISION_RISK,
                                validity_duration=10,
                                event_position=event_position,
                                notify_vehicles=notify,
                            )

                            self.last_denm_time = now

                    if self.avoidance_active:
                        if self.vehicle.name == self.yield_vehicle_name:
                            if distance > YIELD_DISTANCE_M:
                                self.vehicle.target_speed_mps = self.vehicle.base_speed_mps * 0.25
                            else:
                                self.vehicle.target_speed_mps = 0.0
                        else:
                            self.vehicle.target_speed_mps = self.vehicle.base_speed_mps * (
                                0.80 if distance > YIELD_DISTANCE_M else 0.60
                            )

                        if distance > CLEAR_DISTANCE_M and not approaching_each_other:
                            print("[OBU1] Perigo passou, retomando velocidade normal\n")
                            self.avoidance_active = False
                            self.yield_vehicle_name = ""
                            self.yield_mode = "stop"
                            self.yield_resume_time = now + 2.0

                    if not self.avoidance_active:
                        if now < self.yield_resume_time:
                            self.vehicle.target_speed_mps = self.vehicle.base_speed_mps * 0.35
                        else:
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
        print(" OBU1 AGENTE DESCENTRALIZADO - ARNACÓ BRAGA")
        print("=" * 60)
        print(f" Station ID: {self.station_id}")
        print(f" Rota: {self.vehicle.start_point} → {self.vehicle.end_point}")
        print(f" Broker: {self.broker}:{self.port}")
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
        threading.Thread(target=self.collision_detection_loop, daemon=True).start()
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
