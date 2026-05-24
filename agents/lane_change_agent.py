"""Decentralized Lane Change Agent (Emergency Corridor)."""

import json
import signal
import time
import math
import random
import paho.mqtt.client as mqtt

import simulator_core as core

RUNNING = True

DENM_CAUSE_EMERGENCY      = 95
DENM_CC_KEY               = "emergencyVehicleApproaching95"
DENM_SUB_CAUSE            = 1

EMERGENCY_DENM_INTERVAL_S = 1.0
YIELD_DISTANCE_THRESHOLD  = 180.0

def signal_handler(_signum, _frame):
    global RUNNING
    RUNNING = False

class LaneChangeAgent:
    def __init__(self, profile: dict):
        self.profile = profile
        self.name = profile.get("name", "car")
        self.station_id = int(profile.get("stationId", 0))
        self.broker = profile.get("broker", "localhost")
        self.port = int(profile.get("port", 1883))
        
        # --- Random Lane Logic ---
        configured_lane = profile.get("lane", "left")
        if configured_lane == "random":
            self.current_lane = random.SystemRandom().choice(["left", "right"])
        else:
            self.current_lane = configured_lane
            
        print(f"[{self.name}] Configured lane: {configured_lane} -> Spawned in lane: {self.current_lane}")
            
        self.is_emergency = profile.get("isEmergency", False)
        
        self.start_point = tuple(profile.get("startPoint"))
        self.end_point = tuple(profile.get("endPoint"))

        self.vehicle = core.VehicleSim(
            name=self.name,
            station_id=self.station_id,
            broker_host=self.broker,
            start_point=self.start_point,
            end_point=self.end_point,
            base_speed_mps=float(profile.get("baseSpeedMps", 6.0)),
            loop_route=False,
        )

        self.base_route = self.vehicle.route.copy()

        shifted_route = self.offset_route(self.base_route, self.current_lane)
        self.vehicle.set_route(shifted_route, keep_current_position=False)

        start_offset = float(profile.get("startOffsetM", 0.0))
        if start_offset > 0.0:
            self._advance_vehicle_by_meters(start_offset)

        self.mqtt_client = mqtt.Client(client_id=f"{self.name}-agent-sub")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.mqtt_client.connect_async(self.broker, self.port, keepalive=60)

        self.peer_clients: list[mqtt.Client] = []
        self.last_published_denm_time = 0.0
        self._denm_seq = 0
        
        # Timer to hold the slow speed while EV passes
        self.yield_until = 0.0

    def offset_route(self, route: list, lane_type: str) -> list:
        if lane_type not in ["left", "right"]:
            return route

        offset_m = 4.5  # Increased slightly for better visual clarity on Leaflet maps
        new_route = []
        R = 6378137.0   
        
        for i in range(len(route)):
            lat, lon = route[i]
            
            if i < len(route) - 1:
                heading = core.bearing_degrees(lat, lon, route[i+1][0], route[i+1][1])
            else:
                heading = core.bearing_degrees(route[i-1][0], route[i-1][1], lat, lon)
            
            if lane_type == "left":
                offset_heading = (heading - 90) % 360
            else:
                offset_heading = (heading + 90) % 360
            
            d_lat = math.cos(math.radians(offset_heading)) * offset_m / R
            d_lon = math.sin(math.radians(offset_heading)) * offset_m / (R * math.cos(math.radians(lat)))
            
            new_lat = lat + math.degrees(d_lat)
            new_lon = lon + math.degrees(d_lon)
            new_route.append((new_lat, new_lon))
            
        return new_route

    def _advance_vehicle_by_meters(self, distance_m: float):
        remaining = distance_m
        for i in range(len(self.vehicle.route) - 1):
            p1 = self.vehicle.route[i]
            p2 = self.vehicle.route[i+1]
            seg_dist = core.haversine_meters(*p1, *p2)
            if remaining <= seg_dist:
                ratio = remaining / max(seg_dist, 0.01)
                self.vehicle.current_lat, self.vehicle.current_lon = core.interpolate(*p1, *p2, ratio)
                self.vehicle.segment_idx = i
                self.vehicle.last_heading_deg = core.bearing_degrees(*p1, *p2)
                self.base_segment_idx = i 
                return
            remaining -= seg_dist
            
        self.vehicle.current_lat, self.vehicle.current_lon = self.vehicle.route[-1]
        self.vehicle.segment_idx = len(self.vehicle.route) - 1
        self.base_segment_idx = len(self.base_route) - 1
        
    def attach_peer_listener(self, host: str, client_id: str):
        c = mqtt.Client(client_id=client_id)
        c.reconnect_delay_set(min_delay=1, max_delay=10)
        c.connect_async(host, self.port, keepalive=60)
        c.loop_start()
        self.peer_clients.append(c)

    @staticmethod
    def _its_timestamp() -> int:
        return int((time.time() - 1072915200) * 1000)

    def _publish_emergency_denm(self):
        self._denm_seq = (self._denm_seq + 1) % 65535

        denm_in = {
            "management": {
                "actionId": {
                    "originatingStationId": self.vehicle.station_id,
                    "sequenceNumber": self._denm_seq,
                },
                "detectionTime": self._its_timestamp(),
                "referenceTime": self._its_timestamp(),
                "eventPosition": {
                    "latitude":  self.vehicle.current_lat,
                    "longitude": self.vehicle.current_lon,
                    "positionConfidenceEllipse": {"semiMajorConfidence": 50, "semiMinorConfidence": 50, "semiMajorOrientation": 0},
                    "altitude": {"altitudeValue": 0, "altitudeConfidence": 1},
                },
                "stationType": 5,
                "validityDuration": 3,
            },
            "situation": {
                "informationQuality": 7,
                "eventType": {"ccAndScc": {DENM_CC_KEY: DENM_SUB_CAUSE}},
            },
        }

        denm_out = {
            "timestamp": time.time(),
            "rssi": -16,
            "stationID": self.vehicle.station_id,
            "newInfo": True,
            "fields": {
                "header": {"protocolVersion": 2, "messageId": 1, "stationId": self.vehicle.station_id},
                "denm": denm_in,
            },
        }

        self.vehicle.client.publish("vanetza/in/denm", json.dumps(denm_in), qos=0)
        out_bytes = json.dumps(denm_out).encode()
        for c in self.peer_clients:
            c.publish("vanetza/out/denm", out_bytes, qos=0)

    @staticmethod
    def _extract_cause(denm):
        event_type = denm.get("situation", {}).get("eventType", {})
        if "causeCode" in event_type: return event_type["causeCode"]
        cc_and_scc = event_type.get("ccAndScc", {})
        for key in cc_and_scc:
            if "emergencyVehicleApproaching" in key: return DENM_CAUSE_EMERGENCY
        return None

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe("vanetza/out/denm")

    def change_to_right_lane(self):
        self.current_lane = "changing"
        
        heading = self.vehicle.last_heading_deg
        R = 6378137.0
        
        merge_fwd_m = 20.0
        d_lat_fwd = math.cos(math.radians(heading)) * merge_fwd_m / R
        d_lon_fwd = math.sin(math.radians(heading)) * merge_fwd_m / (R * math.cos(math.radians(self.vehicle.current_lat)))
        fwd_lat = self.vehicle.current_lat + math.degrees(d_lat_fwd)
        fwd_lon = self.vehicle.current_lon + math.degrees(d_lon_fwd)

        lane_width = 4.5
        offset_heading = (heading + 90) % 360
        d_lat_right = math.cos(math.radians(offset_heading)) * lane_width / R
        d_lon_right = math.sin(math.radians(offset_heading)) * lane_width / (R * math.cos(math.radians(fwd_lat)))
        
        merge_lat = fwd_lat + math.degrees(d_lat_right)
        merge_lon = fwd_lon + math.degrees(d_lon_right)
        merge_point = (merge_lat, merge_lon)

        current_base_idx = getattr(self, 'base_segment_idx', self.vehicle.segment_idx)
        next_idx = min(current_base_idx + 1, len(self.base_route) - 1)
        remaining_base = self.base_route[next_idx:]
        shifted_right = self.offset_route(remaining_base, "right")
        
        new_route = [(self.vehicle.current_lat, self.vehicle.current_lon), merge_point] + shifted_right
        
        self.vehicle.set_route(new_route, keep_current_position=True)
        
        # Merge with momentum (70% speed instead of 40%)
        self.vehicle.target_speed_mps = self.vehicle.base_speed_mps * 0.7

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            denm    = payload.get("fields", {}).get("denm")
            if not denm: return

            origin = denm.get("management", {}).get("actionId", {}).get("originatingStationId")
            if origin == self.vehicle.station_id: return

            cause = self._extract_cause(denm)
            if cause == DENM_CAUSE_EMERGENCY and not self.is_emergency:
                ev_lat = denm["management"]["eventPosition"]["latitude"]
                ev_lon = denm["management"]["eventPosition"]["longitude"]
                
                dist = core.haversine_meters(self.vehicle.current_lat, self.vehicle.current_lon, ev_lat, ev_lon)
                bearing_to_ev = core.bearing_degrees(self.vehicle.current_lat, self.vehicle.current_lon, ev_lat, ev_lon)
                heading_diff = core.heading_delta_degrees(self.vehicle.last_heading_deg, bearing_to_ev)
                is_behind = abs(heading_diff) > 90.0
                
                # If EV is behind us and close, we react
                if is_behind and dist <= YIELD_DISTANCE_THRESHOLD:
                    # Keep refreshing the yield timer as long as we get DENMs from behind
                    self.yield_until = time.time() + 6.0 
                    
                    if self.current_lane == "left":
                        self.change_to_right_lane()
                    elif self.current_lane == "right":
                        # If we are already on the right, just slow down to let them pass safely
                        self.vehicle.target_speed_mps = self.vehicle.base_speed_mps * 0.4
                        
        except Exception as e:
            pass

    def step_loop(self):
        while RUNNING:
            try:
                self.vehicle.step_and_publish(core.TICK_SECONDS)
                
                if self.current_lane != "changing" and getattr(self, 'base_segment_idx', None) is not None:
                    if self.vehicle.segment_idx > self.base_segment_idx:
                        self.base_segment_idx = self.vehicle.segment_idx
                
                # Speed Recovery Logic:
                # If the timer has expired (meaning the EV passed us and is now in front), resume normal speed
                if not self.is_emergency and self.yield_until > 0:
                    if time.time() > self.yield_until:
                        self.vehicle.target_speed_mps = self.vehicle.base_speed_mps
                        if self.current_lane == "changing":
                            self.current_lane = "right" # Mark merge as complete
                        self.yield_until = 0.0

                if self.is_emergency:
                    now = time.time()
                    if now - self.last_published_denm_time > EMERGENCY_DENM_INTERVAL_S:
                        self._publish_emergency_denm()
                        self.last_published_denm_time = now
                time.sleep(core.TICK_SECONDS)
            except Exception as e:
                time.sleep(core.TICK_SECONDS)

    def run(self):
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        self.mqtt_client.loop_start()
        try:
            self.step_loop()
        finally:
            self.vehicle.close()
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            for c in self.peer_clients:
                c.loop_stop()
                c.disconnect()