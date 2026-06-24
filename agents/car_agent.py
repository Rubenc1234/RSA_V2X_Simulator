"""Shared CarAgent used by OBU containers.

This centralises the agent logic so each OBU container can be a thin
bootstrap that loads a profile and runs the same implementation.

Supported scenarios (selected via profile fields, no class swap needed):
  * Default collision-risk / accident behaviour (scenarios 1, 3, ...)
      - Bilateral collision detection, accident DENMs, optional reroute.
  * Emergency-corridor / lane-change behaviour (scenario 2)
      - Spawn the vehicle offset to a left/right lane along its route.
      - If `isEmergency=True`, periodically emit a DENM with cause 95
        (emergencyVehicleApproaching).
      - If `reactToEmergencyVehicle=True` (auto when `lane` is set on a
        non-emergency car), react to DENMs arriving from behind by
        merging to the right lane (when currently on the left) or
        slowing down (when already on the right).
"""
from __future__ import annotations

import json
import math
import random
import time
import threading
from types import SimpleNamespace
from typing import Optional

import paho.mqtt.client as mqtt

import simulator_core as core
from simulator_core import (
    VehicleSim,
    TICK_SECONDS,
    DENM_CAUSE_ACCIDENT,
    DENM_CAUSE_COLLISION_RISK,
    DENM_CAUSE_EMERGENCY,
    WARNING_DISTANCE_M,
    YIELD_DISTANCE_M,
    CLEAR_DISTANCE_M,
    choose_yield_vehicle,
    vehicles_are_approaching_each_other,
    vehicles_share_same_road_and_opposite_direction,
    vehicle_nearest_exit,
    haversine_meters,
    bearing_degrees,
    heading_delta_degrees,
    interpolate,
)


# Earth radius (semi-major axis, WGS-84) used for local metres-to-degrees offsets.
_EARTH_RADIUS_M = 6378137.0


class CarAgent:
    def __init__(self, profile: dict):
        self.profile = profile
        self.name = profile.get("name", "car")
        self.station_id = int(profile.get("stationId", 0))
        self.broker = profile.get("broker", "localhost")
        self.port = int(profile.get("port", 1883))

        self.vehicle = VehicleSim(
            name=self.name,
            station_id=self.station_id,
            broker_host=self.broker,
            start_point=tuple(profile.get("startPoint")),
            end_point=tuple(profile.get("endPoint")),
            base_speed_mps=float(profile.get("baseSpeedMps", 5.0)),
            loop_route=bool(profile.get("loopRoute", True)),
        )

        # MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.peer_clients = []

        # state
        self.other_vehicle_data = {}
        self.avoidance_active = False
        self.yield_mode = "stop"
        self.yield_vehicle_name = ""
        self.denm_hold_until = 0.0

        # publishing/receiving timestamps to avoid self-suppression
        self.last_published_time = 0.0
        self.last_published_event_pos = None
        self.last_received_denm_time = 0.0
        self._last_accident_denm = 0.0

        # behavior flags ---------------------------------------------------
        self.reroute_on_accident = bool(profile.get("rerouteOnAccident", False))
        self.reroute_prefer_alternative = bool(profile.get("reroutePreferAlternative", True))
        self.emit_collision_risk_denm = bool(profile.get("emitCollisionRiskDenm", True))
        self.hard_stop_on_accident = bool(profile.get("hardStopOnAccident", True))
        self.accident_stop_distance_m = float(profile.get("accidentStopDistanceM", 140.0))
        self.accident_ignore_distance_m = float(profile.get("accidentIgnoreDistanceM", 260.0))
        self.denm_stop_distance_m = float(profile.get("denmStopDistanceM", 110.0))
        self.denm_slowdown_distance_m = float(profile.get("denmSlowdownDistanceM", 180.0))
        self.denm_ignore_distance_m = float(profile.get("denmIgnoreDistanceM", 320.0))
        self.incident_on_arrival = bool(profile.get("incidentOnArrival", False))
        self.incident_after_seconds = profile.get("incidentAfterSeconds")
        self.incident_cause_code = int(profile.get("incidentCauseCode", DENM_CAUSE_ACCIDENT))
        self.incident_sub_cause_code = int(profile.get("incidentSubCauseCode", 0))
        # Default accident validity shortened to 5s for quicker map expiry
        self.incident_validity_duration = int(profile.get("incidentValidityDuration", 10))
        self.incident_reset_seconds = float(profile.get("incidentResetSeconds", 12.0))
        self.incident_started_at = time.time()
        self.incident_triggered_at = 0.0
        self.incident_triggered = False

        # ── Lane-change / emergency-corridor behaviour ────────────────────
        self.lane = profile.get("lane")  # "left" | "right" | "random" | None
        self.lane_offset_m = float(profile.get("laneOffsetM", 4.5))
        self.start_offset_m = float(profile.get("startOffsetM", 0.0))
        self.is_emergency = bool(profile.get("isEmergency", False))
        self.emergency_denm_interval_s = float(profile.get("emergencyDenmIntervalS", 1.0))
        self.emergency_denm_validity_s = int(profile.get("emergencyDenmValidityS", 3))
        self.emergency_denm_sub_cause = int(profile.get("emergencyDenmSubCause", 1))
        self.yield_distance_threshold_m = float(profile.get("yieldDistanceThresholdM", 180.0))
        self.yield_duration_s = float(profile.get("yieldDurationS", 6.0))

        # When a lane is configured we're in corridor mode → derive sensible
        # defaults so scenario configs stay minimal.
        lane_mode_active = self.lane is not None
        self.enable_collision_detection = bool(
            profile.get("enableCollisionDetection", not lane_mode_active)
        )
        self.react_to_emergency_vehicle = bool(
            profile.get(
                "reactToEmergencyVehicle",
                lane_mode_active and not self.is_emergency,
            )
        )

        # Resolve random lane choice once at spawn
        if self.lane == "random":
            self.current_lane = random.SystemRandom().choice(["left", "right"])
            print(f"[{self.name}] Configured lane: random -> spawned in lane: {self.current_lane}")
        else:
            self.current_lane = self.lane  # "left" | "right" | None

        # Save the un-shifted (base) route before applying lane offset, so we
        # can splice the right-shifted remainder during a dynamic lane change.
        self.base_route = list(self.vehicle.route)
        self.base_segment_idx = 0

        if self.current_lane in ("left", "right"):
            shifted = self._offset_route(self.base_route, self.current_lane)
            self.vehicle.set_route(shifted, keep_current_position=False)
            print(f"[{self.name}] Lane offset applied: {self.current_lane} ({self.lane_offset_m:.1f}m)")

        # Stagger spawns by advancing some distance along the route
        if self.start_offset_m > 0.0:
            self._advance_vehicle_by_meters(self.start_offset_m)
            print(f"[{self.name}] Advanced {self.start_offset_m:.1f}m along route at spawn")

        # Emergency-corridor runtime state
        self.yield_until = 0.0
        self._emergency_last_published = 0.0

    # ───────────────────────── Incident DENM ─────────────────────────────
    def trigger_accident_denm(self, reason: str) -> None:
        if self.incident_triggered:
            return

        self.incident_triggered = True
        self.incident_triggered_at = time.time()
        self.vehicle.hard_stop = True
        self.vehicle.current_speed_mps = 0.0
        self.vehicle.target_speed_mps = 0.0
        self.avoidance_active = True
        self.yield_vehicle_name = self.vehicle.name
        self.yield_mode = "stop"
        self.denm_hold_until = time.time() + 9999.0

        event_position = (self.vehicle.current_lat, self.vehicle.current_lon)
        print(f"[{self.name}] INCIDENTE {reason} -> veículo parado e DENM de acidente emitido")

        notify = [SimpleNamespace(client=self.vehicle.client)]
        if self.peer_clients:
            notify.extend(SimpleNamespace(client=client) for client in self.peer_clients)

        core.publish_denm(
            self.vehicle,
            cause_code=self.incident_cause_code,
            sub_cause_code=self.incident_sub_cause_code,
            validity_duration=self.incident_validity_duration,
            event_position=event_position,
            notify_vehicles=notify,
        )

    def reset_after_accident(self) -> None:
        self.vehicle.hard_stop = False
        self.vehicle._restart_route()
        self.vehicle.current_speed_mps = self.vehicle.base_speed_mps
        self.vehicle.target_speed_mps = self.vehicle.base_speed_mps
        self.avoidance_active = False
        self.yield_vehicle_name = ""
        self.yield_mode = "stop"
        self.denm_hold_until = 0.0
        self.incident_triggered = False
        self.incident_triggered_at = 0.0
        print(f"[{self.name}] ACCIDENT RESET -> rota reiniciada e veículo pronto para repetir")

    def maybe_reset_after_accident(self) -> None:
        if not self.vehicle.hard_stop or not self.incident_triggered:
            return
        if self.incident_triggered_at <= 0.0:
            return
        if (time.time() - self.incident_triggered_at) >= self.incident_reset_seconds:
            self.reset_after_accident()

    # ────────────────────────── Peer broker hookup ───────────────────────
    def attach_peer_listener(self, broker_host: str, client_suffix: str):
        client = mqtt.Client(client_id=client_suffix)
        client.on_message = self.on_message
        try:
            client.connect(broker_host, self.port, keepalive=60)
        except Exception as e:
            print(f"[{self.name}] Erro ao ligar ao broker peer {broker_host}:{self.port}: {e}")
            return
        client.subscribe("vanetza/out/cam")
        client.subscribe("vanetza/out/denm")
        client.loop_start()
        self.peer_clients.append(client)
        print(f"[{self.name}] Subscribed to peer broker {broker_host}")

    # ────────────────────────── MQTT callbacks ───────────────────────────
    def on_connect(self, client, userdata, flags, rc):
        print(f"[{self.name}] Conectado ao MQTT broker (rc={rc})")
        self.mqtt_client.subscribe("vanetza/out/cam")
        self.mqtt_client.subscribe("vanetza/out/denm")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            station_id = payload.get("fields", {}).get("header", {}).get("stationId")

            if "cam" in msg.topic or payload.get("fields", {}).get("cam") is not None:
                if station_id is None or station_id == self.station_id:
                    return
                candidate_cam = payload.get("fields", {}).get("cam")
                pos = self.extract_position_from_cam(candidate_cam)
                if pos:
                    self.other_vehicle_data[station_id] = candidate_cam
                    print(f"[{self.name}] Recebi CAM de station_id={station_id}")

            elif "denm" in msg.topic or payload.get("fields", {}).get("denm") is not None:
                # Ignore our own DENMs echoed back from the broker
                origin = (
                    payload.get("fields", {})
                    .get("denm", {})
                    .get("management", {})
                    .get("actionId", {})
                    .get("originatingStationId")
                )
                if origin == self.station_id:
                    return
                self.last_received_denm_time = time.time()
                denm_pos = payload.get("fields", {}).get("denm", {}).get("management", {}).get("eventPosition")
                print(f"[{self.name}] Recebi DENM: {denm_pos}")
                self.apply_denm_reaction(payload)

        except Exception as e:
            print(f"[{self.name}] Erro ao processar mensagem: {e}")

    def extract_position_from_cam(self, cam_payload):
        try:
            basic = cam_payload["camParameters"]["basicContainer"]["referencePosition"]
            return (basic["latitude"], basic["longitude"])
        except Exception:
            return None

    def extract_heading_from_cam(self, cam_payload):
        try:
            high = cam_payload["camParameters"]["highFrequencyContainer"]["basicVehicleContainerHighFrequency"]
            return high["heading"]["headingValue"]
        except Exception:
            return 0.0

    def build_peer_vehicle_view(self) -> Optional[SimpleNamespace]:
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

    # ─────────────────────── DENM reaction logic ─────────────────────────
    def apply_denm_reaction(self, payload):
        denm = payload.get("fields", {}).get("denm", {})
        event_position = denm.get("management", {}).get("eventPosition", {})
        event_type = denm.get("situation", {}).get("eventType", {})
        cause_code = event_type.get("causeCode")
        cc_and_scc = event_type.get("ccAndScc", {})

        # Emergency-vehicle DENM — handled by lane-change/yield logic only.
        is_emergency_vehicle = (
            cause_code == DENM_CAUSE_EMERGENCY
            or any("emergencyVehicleApproaching" in k for k in cc_and_scc)
        )
        if is_emergency_vehicle:
            if self.react_to_emergency_vehicle and not self.is_emergency:
                self._react_to_emergency_vehicle(event_position)
            return  # Emergency DENMs never feed the collision/accident path

        peer_vehicle = self.build_peer_vehicle_view()
        if not peer_vehicle:
            return

        is_accident = cause_code == DENM_CAUSE_ACCIDENT or "accident2" in cc_and_scc
        event_lat = event_position.get("latitude")
        event_lon = event_position.get("longitude")

        if not is_accident:
            if event_lat is not None and event_lon is not None:
                event_distance = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, event_lat, event_lon)
            else:
                event_distance = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, peer_vehicle.current_lat, peer_vehicle.current_lon)

            if event_distance > self.denm_ignore_distance_m:
                print(f"[{self.name}] DENM de collision risk muito longe ({event_distance:.1f}m) -> ignorado")
                return

            self.avoidance_active = True
            self.yield_vehicle_name = choose_yield_vehicle(self.vehicle, peer_vehicle).name
            self.denm_hold_until = time.time() + (3.0 if event_distance > self.denm_slowdown_distance_m else 8.0)

            if event_distance > self.denm_slowdown_distance_m:
                self.yield_mode = "slow"
                self.vehicle.target_speed_mps = max(1.0, self.vehicle.base_speed_mps * 0.75)
                print(f"[{self.name}] DENM de collision risk distante ({event_distance:.1f}m) -> abrandar")
            elif event_distance > self.denm_stop_distance_m:
                self.yield_mode = "slow"
                if self.vehicle.name == self.yield_vehicle_name:
                    self.vehicle.target_speed_mps = max(0.5, self.vehicle.base_speed_mps * 0.35)
                else:
                    self.vehicle.target_speed_mps = max(0.8, self.vehicle.base_speed_mps * 0.6)
                print(f"[{self.name}] DENM de collision risk intermédio ({event_distance:.1f}m) -> abrandar mais")
            else:
                self.yield_mode = "stop"
                if self.vehicle.name == self.yield_vehicle_name:
                    self.vehicle.current_speed_mps = 0.0
                    self.vehicle.target_speed_mps = 0.0
                    print(f"[{self.name}] DENM de collision risk muito perto ({event_distance:.1f}m) -> parar")
                else:
                    self.vehicle.target_speed_mps = max(0.6, self.vehicle.base_speed_mps * 0.5)
                    print(f"[{self.name}] DENM de collision risk muito perto ({event_distance:.1f}m) -> outro veículo para, este abranda")
            return

        if is_accident:
            if event_lat is not None and event_lon is not None:
                event_distance = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, event_lat, event_lon)
            else:
                event_distance = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, peer_vehicle.current_lat, peer_vehicle.current_lon)

            if event_distance > self.accident_ignore_distance_m:
                print(f"[{self.name}] ACIDENTE DETETADO a {event_distance:.1f}m -> ignorado")
                return

            if event_distance > self.accident_stop_distance_m:
                self.avoidance_active = True
                self.yield_vehicle_name = self.vehicle.name
                self.yield_mode = "slow"
                self.denm_hold_until = time.time() + 8.0
                self.vehicle.target_speed_mps = max(0.6, self.vehicle.base_speed_mps * 0.35)
                print(f"[{self.name}] ACIDENTE DETETADO a {event_distance:.1f}m -> abrandar")
                return

            if self.hard_stop_on_accident:
                self.vehicle.current_speed_mps = 0.0
                self.vehicle.target_speed_mps = 0.0
                self.vehicle.hard_stop = True
                self.avoidance_active = True
                self.yield_vehicle_name = self.vehicle.name
                self.yield_mode = "stop"
                self.incident_triggered = True
                self.incident_triggered_at = time.time()
                self.denm_hold_until = time.time() + 9999.0
                print(f"[{self.name}] ACIDENTE DETETADO -> veículo parado (hard stop)")
                return
            elif self.reroute_on_accident:
                dest = self.vehicle.end_point
                new_route = self.vehicle.replan_route_to(dest[0], dest[1], use_alternative=self.reroute_prefer_alternative)
                self.vehicle.target_speed_mps = self.vehicle.base_speed_mps
                self.avoidance_active = False
                self.yield_vehicle_name = ""
                self.yield_mode = "reroute"
                self.denm_hold_until = 0.0
                print(f"[{self.name}] ACIDENTE DETETADO -> nova rota calculada ({len(new_route)} waypoints)")
                return

        # Soft-yield behaviour for other DENM types
        yield_vehicle = choose_yield_vehicle(self.vehicle, peer_vehicle)
        self.yield_vehicle_name = yield_vehicle.name
        self.avoidance_active = True
        now = time.time()
        if event_lat is not None and event_lon is not None:
            event_distance = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, event_lat, event_lon)
        else:
            event_distance = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, peer_vehicle.current_lat, peer_vehicle.current_lon)

        self.yield_mode = "soft"
        self.denm_hold_until = time.time() + 6.0

        slow_factor_yield = 0.35
        slow_factor_other = 0.6

        if self.vehicle.name == self.yield_vehicle_name:
            self.vehicle.target_speed_mps = max(0.5, self.vehicle.base_speed_mps * slow_factor_yield)
        else:
            if event_distance <= YIELD_DISTANCE_M:
                self.vehicle.target_speed_mps = max(0.6, self.vehicle.base_speed_mps * (slow_factor_other - 0.15))
            else:
                self.vehicle.target_speed_mps = max(0.8, self.vehicle.base_speed_mps * slow_factor_other)

        print(f"[{self.name}] DENM recebido -> {self.yield_vehicle_name} cede ({self.yield_mode}); evento_dist={event_distance:.1f}m alvo={self.vehicle.target_speed_mps:.1f}m/s")

    # ─────────────── Emergency-corridor / lane-change helpers ────────────
    def _offset_route(self, route, lane_type: str):
        """Return a copy of `route` shifted laterally by `self.lane_offset_m`."""
        if lane_type not in ("left", "right") or len(route) < 2:
            return list(route)

        offset_m = self.lane_offset_m
        new_route = []
        for i in range(len(route)):
            lat, lon = route[i]
            if i < len(route) - 1:
                heading = bearing_degrees(lat, lon, route[i + 1][0], route[i + 1][1])
            else:
                heading = bearing_degrees(route[i - 1][0], route[i - 1][1], lat, lon)

            offset_heading = (heading - 90) % 360 if lane_type == "left" else (heading + 90) % 360

            d_lat = math.cos(math.radians(offset_heading)) * offset_m / _EARTH_RADIUS_M
            d_lon = (
                math.sin(math.radians(offset_heading)) * offset_m
                / (_EARTH_RADIUS_M * math.cos(math.radians(lat)))
            )
            new_route.append((lat + math.degrees(d_lat), lon + math.degrees(d_lon)))
        return new_route

    def _advance_vehicle_by_meters(self, distance_m: float) -> None:
        """Move the vehicle forward along its current route by `distance_m`."""
        remaining = distance_m
        for i in range(len(self.vehicle.route) - 1):
            p1 = self.vehicle.route[i]
            p2 = self.vehicle.route[i + 1]
            seg_dist = haversine_meters(*p1, *p2)
            if remaining <= seg_dist:
                ratio = remaining / max(seg_dist, 0.01)
                self.vehicle.current_lat, self.vehicle.current_lon = interpolate(*p1, *p2, ratio)
                self.vehicle.segment_idx = i
                self.vehicle.last_heading_deg = bearing_degrees(*p1, *p2)
                self.base_segment_idx = i
                return
            remaining -= seg_dist

        self.vehicle.current_lat, self.vehicle.current_lon = self.vehicle.route[-1]
        self.vehicle.segment_idx = len(self.vehicle.route) - 1
        self.base_segment_idx = max(len(self.base_route) - 1, 0)

    def _change_to_right_lane(self) -> None:
        """Splice in a smooth merge from current pos to the right-shifted base route.

        The merge waypoint is found by walking forward **along the base
        route** by `merge_distance_m`, then shifting that point right by
        `lane_offset_m`. Only base waypoints that fall after that merge
        point are kept in the new route, so the trajectory is always
        strictly forward — no reverse motion when the next OSRM waypoint
        happens to be close to the current position.
        """
        if self.current_lane in ("right", "changing"):
            return

        self.current_lane = "changing"

        cur_lat, cur_lon = self.vehicle.current_lat, self.vehicle.current_lon
        merge_distance_m = 20.0

        # Approximate how far along the current base segment we are. The
        # lateral lane offset (~4.5 m) is small compared to typical segment
        # lengths, so projecting straight-line distance onto the segment
        # direction is accurate enough.
        seg_start = self.base_route[self.base_segment_idx]
        next_base_idx = min(self.base_segment_idx + 1, len(self.base_route) - 1)
        seg_end = self.base_route[next_base_idx]
        seg_len = max(haversine_meters(*seg_start, *seg_end), 0.01)

        seg_heading = bearing_degrees(*seg_start, *seg_end)
        bearing_to_cur = bearing_degrees(*seg_start, cur_lat, cur_lon)
        raw_distance = haversine_meters(*seg_start, cur_lat, cur_lon)
        along_distance = raw_distance * math.cos(
            math.radians(heading_delta_degrees(seg_heading, bearing_to_cur))
        )
        along_distance = min(max(along_distance, 0.0), seg_len)

        # Walk forward along the base route until we have travelled
        # along_distance + merge_distance_m from base_route[base_segment_idx].
        target_dist = along_distance + merge_distance_m
        accumulated = 0.0
        merge_point: Optional[tuple] = None
        next_idx_after_merge = len(self.base_route)

        for i in range(self.base_segment_idx, len(self.base_route) - 1):
            p1 = self.base_route[i]
            p2 = self.base_route[i + 1]
            s_dist = max(haversine_meters(*p1, *p2), 0.01)
            if accumulated + s_dist >= target_dist:
                ratio = (target_dist - accumulated) / s_dist
                mp_lat, mp_lon = interpolate(*p1, *p2, ratio)
                # Shift this point perpendicular to its own base segment.
                h = bearing_degrees(*p1, *p2)
                offset_h = (h + 90) % 360
                d_lat = math.cos(math.radians(offset_h)) * self.lane_offset_m / _EARTH_RADIUS_M
                d_lon = (
                    math.sin(math.radians(offset_h)) * self.lane_offset_m
                    / (_EARTH_RADIUS_M * math.cos(math.radians(mp_lat)))
                )
                merge_point = (mp_lat + math.degrees(d_lat), mp_lon + math.degrees(d_lon))
                next_idx_after_merge = i + 1
                break
            accumulated += s_dist

        if merge_point is None:
            # Less than merge_distance_m of base route remains — just rejoin
            # the right lane from the next base waypoint without an extra
            # merge anchor.
            remaining_base = self.base_route[self.base_segment_idx + 1:]
            if not remaining_base:
                return
            shifted_right = self._offset_route(remaining_base, "right")
            new_route = [(cur_lat, cur_lon)] + shifted_right
        else:
            remaining_base = self.base_route[next_idx_after_merge:]
            shifted_right = self._offset_route(remaining_base, "right")
            new_route = [(cur_lat, cur_lon), merge_point] + shifted_right

        if len(new_route) < 2:
            return

        self.vehicle.set_route(new_route, keep_current_position=True)
        # Merge with momentum (70% of base speed) rather than crawling.
        self.vehicle.target_speed_mps = self.vehicle.base_speed_mps * 0.7

    def _publish_emergency_denm(self) -> None:
        notify = [SimpleNamespace(client=self.vehicle.client)]
        if self.peer_clients:
            notify.extend(SimpleNamespace(client=c) for c in self.peer_clients)

        core.publish_denm(
            self.vehicle,
            cause_code=DENM_CAUSE_EMERGENCY,
            sub_cause_code=self.emergency_denm_sub_cause,
            validity_duration=self.emergency_denm_validity_s,
            event_position=(self.vehicle.current_lat, self.vehicle.current_lon),
            notify_vehicles=notify,
        )

    def _react_to_emergency_vehicle(self, event_position: dict) -> None:
        ev_lat = event_position.get("latitude")
        ev_lon = event_position.get("longitude")
        if ev_lat is None or ev_lon is None:
            return

        dist = haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, ev_lat, ev_lon)
        bearing_to_ev = bearing_degrees(self.vehicle.current_lat, self.vehicle.current_lon, ev_lat, ev_lon)
        heading_diff = heading_delta_degrees(self.vehicle.last_heading_deg, bearing_to_ev)
        is_behind = abs(heading_diff) > 90.0

        if not (is_behind and dist <= self.yield_distance_threshold_m):
            return

        # Refresh the yield timer while the EV keeps signalling from behind.
        self.yield_until = time.time() + self.yield_duration_s

        if self.current_lane == "left":
            self._change_to_right_lane()
            print(f"[{self.name}] EV approaching from behind ({dist:.1f}m) -> merging right")
        elif self.current_lane == "right":
            # Already on the right; just slow down to let the EV pass.
            self.vehicle.target_speed_mps = self.vehicle.base_speed_mps * 0.4
            print(f"[{self.name}] EV approaching from behind ({dist:.1f}m) -> slowing down on right lane")

    # ─────────────────────────── Main loops ──────────────────────────────
    def publish_cam_loop(self):
        print(f"[{self.name}] Iniciando publicação de CAM ({1.0 / TICK_SECONDS:.0f} Hz)")
        iteration = 0
        while True:
            try:
                self.maybe_reset_after_accident()
                self.vehicle.step_and_publish(TICK_SECONDS)

                # Keep base_segment_idx in sync while on the original lane,
                # so a later _change_to_right_lane can splice correctly.
                if (self.current_lane in ("left", "right")
                        and self.vehicle.segment_idx > self.base_segment_idx):
                    self.base_segment_idx = self.vehicle.segment_idx

                # Yield-timer recovery: once the EV has passed, resume speed.
                if not self.is_emergency and self.yield_until > 0.0:
                    if time.time() > self.yield_until:
                        self.vehicle.target_speed_mps = self.vehicle.base_speed_mps
                        if self.current_lane == "changing":
                            self.current_lane = "right"
                        self.yield_until = 0.0

                # Emergency vehicles broadcast a DENM at the configured cadence.
                if self.is_emergency:
                    now = time.time()
                    if now - self._emergency_last_published >= self.emergency_denm_interval_s:
                        self._publish_emergency_denm()
                        self._emergency_last_published = now
                
                if self.incident_triggered:
                    print("ACCIDENT ACTIVE")
                    now = time.time()

                    if now - self._last_accident_denm >= 2.0:
                        print("SENDING PERIODIC DENM")
                        notify = [SimpleNamespace(client=self.vehicle.client)]

                        if self.peer_clients:
                            notify.extend(
                                SimpleNamespace(client=c)
                                for c in self.peer_clients
                            )

                        core.publish_denm(
                            self.vehicle,
                            cause_code=self.incident_cause_code,
                            sub_cause_code=self.incident_sub_cause_code,
                            validity_duration=self.incident_validity_duration,
                            event_position=(
                                self.vehicle.current_lat,
                                self.vehicle.current_lon
                            ),
                            notify_vehicles=notify,
                        )

                        self._last_accident_denm = now

                if not self.incident_triggered:
                    if self.incident_after_seconds is not None and (time.time() - self.incident_started_at) >= float(self.incident_after_seconds):
                        self.trigger_accident_denm("SIMULADO")
                    elif self.incident_on_arrival and self.vehicle.segment_idx >= len(self.vehicle.route) - 1:
                        self.trigger_accident_denm("(on arrival)")

                iteration += 1
                if iteration % 5 == 0:
                    print(f"[{self.name}] CAM: lat={self.vehicle.current_lat:.6f} lon={self.vehicle.current_lon:.6f} speed={self.vehicle.current_speed_mps:.1f}m/s")
                time.sleep(TICK_SECONDS)
            except Exception as e:
                print(f"[{self.name}] Erro ao publicar CAM: {e}")
                time.sleep(TICK_SECONDS)

    def collision_detection_loop(self):
        if not self.enable_collision_detection:
            print(f"[{self.name}] Collision detection disabled for this scenario")
            return
        print(f"[{self.name}] Iniciando detecção de colisão")
        while True:
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
                        print(f"[{self.name}] Distância para peer: {distance:.1f}m (bearing: {bearing_to_peer:.1f}°)")

                    now = time.time()
                    risk_window = ((same_road_opposite and approaching_each_other) or (close_range_risk and approaching_each_other))
                    if risk_window and distance < WARNING_DISTANCE_M:
                        if not self.avoidance_active and now - self.last_published_time > 5.0:
                            yield_vehicle = choose_yield_vehicle(self.vehicle, peer_vehicle)
                            self.yield_vehicle_name = yield_vehicle.name
                            self.yield_mode = "stop"
                            self.avoidance_active = True
                            self.denm_hold_until = now + 4.0

                            yield_side = vehicle_nearest_exit(yield_vehicle)
                            yield_distance = (yield_vehicle.distance_from_route_start_m() if yield_side == "start" else yield_vehicle.distance_to_route_end_m())

                            print(f"\n[{self.name}] AVISO DE COLISÃO")
                            print(f"       Distância: {distance:.1f}m < {WARNING_DISTANCE_M}m")
                            print(f"       {self.yield_vehicle_name} deve ceder ({self.yield_mode})")
                            print(f"       Margem livre na rota: {yield_distance:.1f}m")

                            notify = [SimpleNamespace(client=self.vehicle.client)]
                            if self.peer_clients:
                                notify.append(SimpleNamespace(client=self.peer_clients[0]))

                            event_position = ((my_lat + peer_lat) / 2.0, (my_lon + peer_lon) / 2.0)

                            publish_ok = True
                            if self.last_published_event_pos is not None and self.last_published_time > 0.0:
                                last_lat, last_lon = self.last_published_event_pos
                                moved = haversine_meters(last_lat, last_lon, event_position[0], event_position[1])
                                if moved < 15.0 and (now - self.last_published_time) < 20.0:
                                    publish_ok = False

                            if publish_ok and self.emit_collision_risk_denm:
                                core.publish_denm(self.vehicle, cause_code=DENM_CAUSE_COLLISION_RISK, validity_duration=10, event_position=event_position, notify_vehicles=notify)
                                self.last_published_event_pos = event_position
                                self.last_published_time = now
                            else:
                                if not publish_ok:
                                    print(f"[{self.name}] Skipping duplicate DENM (moved={moved:.1f}m recent={now - self.last_published_time:.1f}s)")

                if self.avoidance_active:
                    slow_factor_yield = 0.35
                    slow_factor_other = 0.6

                    if self.vehicle.name == self.yield_vehicle_name:
                        self.vehicle.target_speed_mps = max(0.5, self.vehicle.base_speed_mps * slow_factor_yield)
                    else:
                        if distance <= YIELD_DISTANCE_M:
                            self.vehicle.target_speed_mps = max(0.6, self.vehicle.base_speed_mps * (slow_factor_other - 0.15))
                        else:
                            self.vehicle.target_speed_mps = max(0.8, self.vehicle.base_speed_mps * slow_factor_other)

                    if ((distance > CLEAR_DISTANCE_M and not approaching_each_other) or time.time() >= self.denm_hold_until):
                        print(f"[{self.name}] Perigo passou, retomando velocidade normal\n")
                        self.avoidance_active = False
                        self.yield_vehicle_name = ""
                        self.yield_mode = "soft"
                        self.denm_hold_until = 0.0

                if not self.avoidance_active and self.yield_until == 0.0:
                    self.vehicle.target_speed_mps = self.vehicle.base_speed_mps

                time.sleep(0.5)
            except Exception as e:
                print(f"[{self.name}] Erro em detecção colisão: {e}")
                time.sleep(0.5)

    def denm_listener_loop(self):
        print(f"[{self.name}] Iniciando listener de DENM")
        first = True
        while True:
            try:
                if not first:
                    time.sleep(1.0)
                first = False
            except Exception as e:
                print(f"[{self.name}] Erro em DENM listener: {e}")
                time.sleep(1.0)

    # ──────────────────────────── Entry point ────────────────────────────
    def run(self):
        print("\n" + "=" * 60)
        print(f" {self.name.upper()} AGENTE DESCENTRALIZADO - inicializando")
        print("=" * 60)
        print(f" Station ID: {self.station_id}")
        print(f" Rota: {self.vehicle.start_point} → {self.vehicle.end_point}")
        print(f" Broker: {self.broker}:{self.port}")
        if self.lane is not None:
            print(f" Lane: {self.current_lane} (offset={self.lane_offset_m:.1f}m)")
        if self.is_emergency:
            print(f" Role: EMERGENCY VEHICLE (DENM cause {DENM_CAUSE_EMERGENCY})")
        if self.react_to_emergency_vehicle:
            print(f" Reacts to emergency vehicles within {self.yield_distance_threshold_m:.0f}m")
        print(f" Collision detection: {'on' if self.enable_collision_detection else 'off'}")
        print("=" * 60 + "\n")

        try:
            self.mqtt_client.connect(self.broker, self.port, keepalive=60)
            print(f"[{self.name}] Conectado ao broker MQTT\n")
        except Exception as e:
            print(f"[{self.name}] Erro ao conectar MQTT: {e}")
            return

        threading.Thread(target=self.publish_cam_loop, daemon=True).start()
        threading.Thread(target=self.collision_detection_loop, daemon=True).start()
        #threading.Thread(target=self.denm_listener_loop, daemon=True).start()

        try:
            self.mqtt_client.loop_forever()
        except KeyboardInterrupt:
            pass
        finally:
            print(f"\n[{self.name}] Encerrando...")
            self.vehicle.close()
            self.mqtt_client.disconnect()
            for client in self.peer_clients:
                client.loop_stop()
                client.disconnect()
