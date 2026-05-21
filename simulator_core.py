"""Shared V2X simulator core for all scenarios."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import paho.mqtt.client as mqtt
import requests

CAM_TOPIC_IN = "vanetza/in/cam"
DENM_TOPIC_IN = "vanetza/in/denm"
DENM_TOPIC_OUT = "vanetza/out/denm"

TICK_HZ = 5.0
TICK_SECONDS = 1.0 / TICK_HZ
OSRM_SERVER = "http://router.project-osrm.org"
WARNING_DISTANCE_M = 80.0
YIELD_DISTANCE_M = 35.0
REVERSE_DISTANCE_M = 20.0
CLEAR_DISTANCE_M = 28.0
LAST_DENM_TIME = 0.0

# ETSI cause codes
DENM_CAUSE_COLLISION_RISK = 26   # used by arnaco_braga proximity scenario
DENM_CAUSE_ACCIDENT = 2          # used by accident scenarios

# calcular distancia entre 2 pontos em metros usando a fórmula de Haversine
def haversine_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
	r = 6371000.0
	phi1 = math.radians(lat1)
	phi2 = math.radians(lat2)
	dphi = math.radians(lat2 - lat1)
	dlambda = math.radians(lon2 - lon1)

	a = (
		math.sin(dphi / 2.0) ** 2
		+ math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
	)
	c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
	return r * c


# calcular a direcao entre 2 pontos em graus
def bearing_degrees(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
	phi1 = math.radians(lat1)
	phi2 = math.radians(lat2)
	dlambda = math.radians(lon2 - lon1)

	x = math.sin(dlambda) * math.cos(phi2)
	y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(
		dlambda
	)

	brng = math.degrees(math.atan2(x, y))
	return (brng + 360.0) % 360.0

# suavizar o movimento entre 2 pontos
def interpolate(
	lat1: float, lon1: float, lat2: float, lon2: float, ratio: float
) -> Tuple[float, float]:
	return (lat1 + (lat2 - lat1) * ratio, lon1 + (lon2 - lon1) * ratio)

# calcular a distancia da rota inicializada pelo OSRM somando as distancias entre os waypoints
def route_length_m(route: List[Tuple[float, float]]) -> float:
	return sum(haversine_meters(*route[i], *route[i + 1]) for i in range(len(route) - 1))

# calcular a menor diferença entre 2 direções em graus, considerando o ciclo de 360°
def heading_delta_degrees(a_deg: float, b_deg: float) -> float:
	"""Return the smallest absolute difference between two headings."""
	return abs(((a_deg - b_deg + 180.0) % 360.0) - 180.0)


def generation_delta_time() -> int:
	"""ETSI generationDeltaTime in milliseconds modulo 65536."""
	return int((time.time() * 1000.0) % 65536)


def get_osrm_route(start_lat: float, start_lon: float, end_lat: float, end_lon: float) -> List[Tuple[float, float]]:
	"""Ask OSRM for a route and return the waypoints as (lat, lon)."""
	try:
		url = f"{OSRM_SERVER}/route/v1/driving/{start_lon},{start_lat};{end_lon},{end_lat}"
		params = {
			"overview": "full",
			"geometries": "geojson",
			"steps": "false",
		}
		response = requests.get(url, params=params, timeout=10)
		response.raise_for_status()
		data = response.json()

		if data.get("code") != "Ok":
			print(f" OSRM error: {data.get('message')}")
			return [(start_lat, start_lon), (end_lat, end_lon)]

		route = data.get("routes", [{}])[0]
		geometry = route.get("geometry", {})
		coordinates = geometry.get("coordinates", [])
		waypoints = [(lat, lon) for lon, lat in coordinates]

		if waypoints:
			print(f"✅ Rota obtida: {len(waypoints)} waypoints")
			return waypoints
		return [(start_lat, start_lon), (end_lat, end_lon)]
	except Exception as e:
		print(f" OSRM request failed: {e}")
		return [(start_lat, start_lon), (end_lat, end_lon)]


def build_cam_payload(lat: float, lon: float, speed_mps: float, heading_deg: float) -> dict:
	"""Build a CAM JSON compatible with Vanetza input examples."""
	return {
		"camParameters": {
			"basicContainer": {
				"stationType": 5,
				"referencePosition": {
					"latitude": lat,
					"longitude": lon,
					"positionConfidenceEllipse": {
						"semiMajorAxisLength": 4095,
						"semiMinorAxisLength": 4095,
						"semiMajorAxisOrientation": 3601,
					},
					"altitude": {
						"altitudeValue": 800001,
						"altitudeConfidence": 15,
					},
				},
			},
			"highFrequencyContainer": {
				"basicVehicleContainerHighFrequency": {
					"heading": {
						"headingValue": round(heading_deg, 2),
						"headingConfidence": 127,
					},
					"speed": {
						"speedValue": round(speed_mps, 2),
						"speedConfidence": 127,
					},
					"driveDirection": 2,
					"vehicleLength": {
						"vehicleLengthValue": 1023,
						"vehicleLengthConfidenceIndication": 4,
					},
					"vehicleWidth": 62,
					"longitudinalAcceleration": {
						"value": 0.0,
						"confidence": 102,
					},
					"curvature": {
						"curvatureValue": 1023,
						"curvatureConfidence": 7,
					},
					"curvatureCalculationMode": 2,
					"yawRate": {
						"yawRateValue": 0.0,
						"yawRateConfidence": 8,
					},
					"accelerationControl": {
						"brakePedalEngaged": False,
						"gasPedalEngaged": False,
						"emergencyBrakeEngaged": False,
						"collisionWarningEngaged": False,
						"accEngaged": False,
						"cruiseControlEngaged": False,
						"speedLimiterEngaged": False,
					},
					"steeringWheelAngle": {
						"steeringWheelAngleValue": 512,
						"steeringWheelAngleConfidence": 127,
					},
				}
			},
		},
		"generationDeltaTime": generation_delta_time(),
	}


@dataclass
class VehicleSim:
	name: str
	station_id: int
	broker_host: str
	start_point: Tuple[float, float]
	end_point: Tuple[float, float]
	base_speed_mps: float
	loop_route: bool = False
	collision_count: int = 0
	total_route_length_m: float = 0.0
	# If provided, skips OSRM entirely and uses these waypoints directly.
	# Use this for short routes (<300m) where OSRM may return a straight line.
	manual_route: Optional[List[Tuple[float, float]]] = field(default=None, repr=False)
	on_denm_received: Optional[Callable] = field(default=None, repr=False)

	def __post_init__(self) -> None:
		self.client = mqtt.Client(client_id=f"sim-pub-{self.station_id}")
		self.client.connect(self.broker_host, 1883, 60)
		self.client.loop_start()

		self._sub_client = mqtt.Client(client_id=f"sim-sub-{self.station_id}")
		self._sub_client.on_message = self._handle_denm
		self._sub_client.connect(self.broker_host, 1883, 60)
		self._sub_client.subscribe(DENM_TOPIC_OUT)
		self._sub_client.loop_start()

		if self.manual_route is not None:
			print(f"[{self.name}] Using manual route ({len(self.manual_route)} waypoints)")
			self.route = self.manual_route
		else:
			print(f"[{self.name}] Pedindo rota OSRM...")
			self.route = get_osrm_route(
				self.start_point[0], self.start_point[1],
				self.end_point[0], self.end_point[1],
			)

		if len(self.route) < 2:
			print(f" {self.name}: rota inválida!")
			self.route = [self.start_point, self.end_point]

		self.total_route_length_m = route_length_m(self.route)
		self.segment_idx = 0
		self.current_lat, self.current_lon = self.route[0]
		self.last_heading_deg = bearing_degrees(*self.route[0], *self.route[1])
		self.current_speed_mps = self.base_speed_mps
		self.target_speed_mps = self.base_speed_mps
		self.in_collision_avoidance = False

	def _handle_denm(self, _client, _userdata, msg) -> None:
		"""Called by paho on the subscriber thread when a DENM arrives."""
		try:
			payload = json.loads(msg.payload)
			# Skip DENMs that originated from this vehicle itself
			origin_id = (
				payload.get("fields", {})
				.get("denm", {})
				.get("management", {})
				.get("actionId", {})
				.get("originatingStationId")
			)
			if origin_id == self.station_id:
				return
			print(f"[{self.name}] DENM received (from stationId={origin_id})")
			if self.on_denm_received is not None:
				self.on_denm_received(self, payload)
		except Exception as e:
			print(f"[{self.name}] DENM parse error: {e}")

	# distancia percorrida, serve para retomar marcha depois de ceder ou recuar
	def distance_from_route_start_m(self) -> float:
		distance = 0.0
		for idx in range(self.segment_idx):
			distance += haversine_meters(*self.route[idx], *self.route[idx + 1])
		distance += haversine_meters(*self.route[self.segment_idx], self.current_lat, self.current_lon)
		return distance

	# quanto falta percorrer para chegar ao destino
	def distance_to_route_end_m(self) -> float:
		return max(self.total_route_length_m - self.distance_from_route_start_m(), 0.0)

	def _restart_route(self) -> None:
		"""Restart route from the first waypoint for continuous movement demos."""
		self.segment_idx = 0
		self.current_lat, self.current_lon = self.route[0]
		self.current_speed_mps = self.base_speed_mps
		self.target_speed_mps = self.base_speed_mps
		if len(self.route) > 1:
			self.last_heading_deg = bearing_degrees(*self.route[0], *self.route[1])

	def step_and_publish(self, dt: float) -> None:
		# ── End-of-route guard: hold position and keep broadcasting ──────
		if self.segment_idx >= len(self.route) - 1:
			if self.loop_route:
				self._restart_route()
				p1 = self.route[self.segment_idx]
				p2 = self.route[self.segment_idx + 1]
				seg_dist = max(haversine_meters(*p1, *p2), 0.01)
				move_dist = self.current_speed_mps * dt
				next_progress = min(max(move_dist / seg_dist, 0.0), 1.0)
				self.current_lat, self.current_lon = interpolate(*p1, *p2, next_progress)
				self.last_heading_deg = bearing_degrees(*p1, *p2)
				cam_payload = build_cam_payload(
					lat=self.current_lat,
					lon=self.current_lon,
					speed_mps=self.current_speed_mps,
					heading_deg=self.last_heading_deg,
				)
				self.client.publish(CAM_TOPIC_IN, json.dumps(cam_payload), qos=0)
				return

			self.current_speed_mps = 0.0
			self.target_speed_mps = 0.0
			self.current_lat, self.current_lon = self.route[-1]
			cam_payload = build_cam_payload(
				lat=self.current_lat,
				lon=self.current_lon,
				speed_mps=0.0,
				heading_deg=self.last_heading_deg,
			)
			self.client.publish(CAM_TOPIC_IN, json.dumps(cam_payload), qos=0)
			return

		p1 = self.route[self.segment_idx]
		p2 = self.route[self.segment_idx + 1]   # no modulo — guarded above
		seg_dist = max(haversine_meters(*p1, *p2), 0.01)
		# Accelerate quickly (0.4), decelerate smoothly (0.15)
		alpha = 0.4 if self.target_speed_mps > self.current_speed_mps else 0.15
		self.current_speed_mps += (self.target_speed_mps - self.current_speed_mps) * alpha
		move_dist = self.current_speed_mps * dt

		# quando do segmento percorreu
		dist_from_p1 = haversine_meters(*p1, self.current_lat, self.current_lon)
		progress = min(max(dist_from_p1 / seg_dist, 0.0), 1.0)
		step_ratio = move_dist / seg_dist
		next_progress = progress + step_ratio

		if move_dist >= 0.0:
			# verifica que vai percorrer demasiado, então avança para o próximo segmento, e calcula o progresso nesse próximo segmento
			while next_progress >= 1.0:
				if self.segment_idx >= len(self.route) - 2:
					if self.loop_route:
						self._restart_route()
						p1 = self.route[self.segment_idx]
						p2 = self.route[self.segment_idx + 1]
						seg_dist = max(haversine_meters(*p1, *p2), 0.01)
						next_progress = min(max(next_progress - 1.0, 0.0), 1.0)
						break

					# Reached the final waypoint — snap, stop, publish, done
					self.segment_idx = len(self.route) - 1
					self.current_lat, self.current_lon = self.route[-1]
					self.current_speed_mps = 0.0
					self.target_speed_mps = 0.0
					cam_payload = build_cam_payload(
						lat=self.current_lat,
						lon=self.current_lon,
						speed_mps=0.0,
						heading_deg=self.last_heading_deg,
					)
					self.client.publish(CAM_TOPIC_IN, json.dumps(cam_payload), qos=0)
					return
				self.segment_idx += 1
				p1 = self.route[self.segment_idx]
				p2 = self.route[self.segment_idx + 1]
				seg_dist = max(haversine_meters(*p1, *p2), 0.01)
				next_progress -= 1.0
			self.current_lat, self.current_lon = interpolate(*p1, *p2, next_progress)
			self.last_heading_deg = bearing_degrees(*p1, *p2)
		else:
			# reversing : verificar que vai recuar demasiado, então volta para o segmento anterior, e calcula o progresso nesse segmento anterior
			while next_progress < 0.0:
				if self.segment_idx == 0:
					# Reached the start of the route — snap, stop, publish, done
					next_progress = 0.0
					self.current_speed_mps = 0.0
					self.target_speed_mps = 0.0
					break
				self.segment_idx -= 1
				p1 = self.route[self.segment_idx]
				p2 = self.route[self.segment_idx + 1]
				seg_dist = max(haversine_meters(*p1, *p2), 0.01)
				next_progress += 1.0
			self.current_lat, self.current_lon = interpolate(*p1, *p2, max(next_progress, 0.0))
			self.last_heading_deg = bearing_degrees(*p2, *p1)

		cam_payload = build_cam_payload(
			lat=self.current_lat,
			lon=self.current_lon,
			speed_mps=self.current_speed_mps,
			heading_deg=self.last_heading_deg,
		)
		self.client.publish(CAM_TOPIC_IN, json.dumps(cam_payload), qos=0)

	def close(self) -> None:
		self.client.loop_stop()
		self.client.disconnect()
		self._sub_client.loop_stop()
		self._sub_client.disconnect()


@dataclass
class RsuSim:
	"""Stationary RSU that monitors CAMs from all vehicle brokers and issues DENMs.

	The RSU has its own broker (192.168.98.10) and a fixed position. It
	subscribes to vanetza/out/cam on every vehicle broker to track positions,
	and publishes DENMs to vanetza/in/denm on its own broker so Vanetza
	broadcasts them over the simulated ITS-G5 network.

	An optional on_cam_received(rsu, obu_name, lat, lon, speed, heading)
	callback lets the scenario loop react without polling.
	"""
	name: str
	station_id: int
	broker_host: str
	position: Tuple[float, float]          # (lat, lon) of the RSU
	on_cam_received: Optional[Callable] = field(default=None, repr=False)

	def __post_init__(self) -> None:
		# Publisher — sends DENMs through Vanetza
		self.client = mqtt.Client(client_id=f"rsu-pub-{self.station_id}")
		self.client.connect(self.broker_host, 1883, 60)
		self.client.loop_start()

		# Track latest known position of each vehicle keyed by obu name
		self.vehicle_positions: dict = {}

		# Subscriber clients keyed by broker host — one per vehicle broker so
		# the RSU sees every OBU's decoded CAMs
		self._sub_clients: List[mqtt.Client] = []

	def subscribe_to_broker(self, broker_host: str, broker_label: str) -> None:
		"""Attach a subscription to a vehicle broker's vanetza/out/cam."""
		sub = mqtt.Client(client_id=f"rsu-sub-{self.station_id}-{broker_label}")
		sub.on_message = self._handle_cam
		sub.connect(broker_host, 1883, 60)
		sub.subscribe("vanetza/out/cam")
		sub.loop_start()
		self._sub_clients.append(sub)
		print(f"[{self.name}] Subscribed to CAMs on {broker_host}")

	def _handle_cam(self, _client, _userdata, msg) -> None:
		try:
			payload = json.loads(msg.payload)
			cam = payload.get("fields", {}).get("cam", {})
			pos = cam.get("camParameters", {}).get("basicContainer", {}).get("referencePosition", {})
			hfc = cam.get("camParameters", {}).get("highFrequencyContainer", {}).get("basicVehicleContainerHighFrequency", {})

			lat = pos.get("latitude")
			lon = pos.get("longitude")
			if lat is None or lon is None or (lat == 40.0 and lon == -8.0):
				return

			station_id = payload.get("fields", {}).get("header", {}).get("stationId")
			speed = hfc.get("speed", {}).get("speedValue", 0)
			heading = hfc.get("heading", {}).get("headingValue", 0)

			obu_name = f"station_{station_id}"
			self.vehicle_positions[obu_name] = {
				"lat": lat, "lon": lon,
				"speed": speed, "heading": heading,
				"station_id": station_id,
				"timestamp": time.time(),
			}

			if self.on_cam_received is not None:
				self.on_cam_received(self, obu_name, lat, lon, speed, heading)
		except Exception as e:
			print(f"[{self.name}] CAM parse error: {e}")

	def distance_to(self, lat: float, lon: float) -> float:
		"""Distance in meters from the RSU to a given coordinate."""
		return haversine_meters(self.position[0], self.position[1], lat, lon)

	def publish_cam(self) -> None:
		"""Broadcast a CAM from the RSU fixed position at zero speed.

		This lets the webapp show the RSU as a stationary marker and keeps the
		RSU visible on the map independently of any DENM activity.
		"""
		cam_payload = {
			"camParameters": {
				"basicContainer": {
					"stationType": 15,  # roadSideUnit
					"referencePosition": {
						"latitude": self.position[0],
						"longitude": self.position[1],
						"positionConfidenceEllipse": {
							"semiMajorAxisLength": 10,
							"semiMinorAxisLength": 10,
							"semiMajorAxisOrientation": 0,
						},
						"altitude": {
							"altitudeValue": 800001,
							"altitudeConfidence": 15,
						},
					},
				},
				"highFrequencyContainer": {
					"basicVehicleContainerHighFrequency": {
						"heading": {"headingValue": 0.0, "headingConfidence": 127},
						"speed": {"speedValue": 0.0, "speedConfidence": 127},
						"driveDirection": 0,
						"vehicleLength": {"vehicleLengthValue": 1023, "vehicleLengthConfidenceIndication": 4},
						"vehicleWidth": 62,
						"longitudinalAcceleration": {"value": 0.0, "confidence": 102},
						"curvature": {"curvatureValue": 0, "curvatureConfidence": 7},
						"curvatureCalculationMode": 2,
						"yawRate": {"yawRateValue": 0.0, "yawRateConfidence": 8},
						"accelerationControl": {
							"brakePedalEngaged": False, "gasPedalEngaged": False,
							"emergencyBrakeEngaged": False, "collisionWarningEngaged": False,
							"accEngaged": False, "cruiseControlEngaged": False,
							"speedLimiterEngaged": False,
						},
						"steeringWheelAngle": {"steeringWheelAngleValue": 0, "steeringWheelAngleConfidence": 127},
					}
				},
			},
			"generationDeltaTime": generation_delta_time(),
		}
		self.client.publish(CAM_TOPIC_IN, json.dumps(cam_payload), qos=0)

	def publish_denm(
		self,
		cause_code: int,
		sub_cause_code: int = 0,
		validity_duration: int = 10,
		notify_vehicles: Optional[List[VehicleSim]] = None,
	) -> None:
		"""Publish a DENM from the RSU position."""
		its_epoch_offset = 1072915200

		def timestamp_its() -> int:
			return int((time.time() - its_epoch_offset) * 1000)

		denm_in_payload = {
			"management": {
				"actionId": {
					"originatingStationId": self.station_id,
					"sequenceNumber": 1,
				},
				"detectionTime": timestamp_its(),
				"referenceTime": timestamp_its(),
				"eventPosition": {
					"latitude": self.position[0],
					"longitude": self.position[1],
					"positionConfidenceEllipse": {
						"semiMajorConfidence": 10,
						"semiMinorConfidence": 10,
						"semiMajorOrientation": 0,
					},
					"altitude": {
						"altitudeValue": 0,
						"altitudeConfidence": 1,
					},
				},
				"stationType": 15,   # roadSideUnit
				"validityDuration": validity_duration,
			},
			"situation": {
				"informationQuality": 7,
				"eventType": {
					"causeCode": cause_code,
					"subCauseCode": sub_cause_code,
				},
			},
		}

		denm_out_payload = {
			"timestamp": time.time(),
			"rssi": -16,
			"stationID": self.station_id,
			"newInfo": True,
			"fields": {
				"header": {
					"protocolVersion": 2,
					"messageId": 1,
					"stationId": self.station_id,
				},
				"denm": denm_in_payload,
			},
		}

		# ITS-G5 path via Vanetza
		self.client.publish(DENM_TOPIC_IN, json.dumps(denm_in_payload), qos=0)

		# Direct path to vehicle brokers for webapp visualization
		for vehicle in (notify_vehicles or []):
			vehicle.client.publish(DENM_TOPIC_OUT, json.dumps(denm_out_payload), qos=0)

		print(
			f"[{self.name}] DENM broadcast "
			f"causeCode={cause_code} subCauseCode={sub_cause_code}"
		)

	def close(self) -> None:
		self.client.loop_stop()
		self.client.disconnect()
		for sub in self._sub_clients:
			sub.loop_stop()
			sub.disconnect()


def detect_collision_risk(v1: VehicleSim, v2: VehicleSim) -> bool:
	distance = haversine_meters(v1.current_lat, v1.current_lon, v2.current_lat, v2.current_lon)
	if distance < 20.0:
		print(f" RISK DETECTED! DIST={distance:.2f}m")
		return True
	return False


def vehicles_share_same_road_and_opposite_direction(v1: VehicleSim, v2: VehicleSim) -> bool:
	start_to_end_1 = bearing_degrees(*v1.start_point, *v1.end_point)
	start_to_end_2 = bearing_degrees(*v2.start_point, *v2.end_point)
	heading_gap = heading_delta_degrees(start_to_end_1, start_to_end_2)
	shared_endpoint_gap = min(
		haversine_meters(*v1.end_point, *v2.start_point),
		haversine_meters(*v1.start_point, *v2.end_point),
	)
	return heading_gap > 150.0 and shared_endpoint_gap < 35.0


def choose_yield_vehicle(v1: VehicleSim, v2: VehicleSim) -> VehicleSim:
	start_1 = v1.distance_from_route_start_m()
	end_1 = v1.distance_to_route_end_m()
	start_2 = v2.distance_from_route_start_m()
	end_2 = v2.distance_to_route_end_m()
	front_cost_1 = min(start_1, end_1)
	front_cost_2 = min(start_2, end_2)
	if abs(front_cost_1 - front_cost_2) < 5.0:
		return v1 if v1.station_id < v2.station_id else v2
	return v1 if front_cost_1 < front_cost_2 else v2


def vehicles_are_approaching_each_other(v1: VehicleSim, v2: VehicleSim) -> bool:
	bearing_1_to_2 = bearing_degrees(v1.current_lat, v1.current_lon, v2.current_lat, v2.current_lon)
	bearing_2_to_1 = bearing_degrees(v2.current_lat, v2.current_lon, v1.current_lat, v1.current_lon)
	heading_gap_1 = heading_delta_degrees(v1.last_heading_deg, bearing_1_to_2)
	heading_gap_2 = heading_delta_degrees(v2.last_heading_deg, bearing_2_to_1)
	return heading_gap_1 < 70.0 and heading_gap_2 < 70.0


def vehicle_nearest_exit(vehicle: VehicleSim) -> str:
	return "start" if vehicle.distance_from_route_start_m() <= vehicle.distance_to_route_end_m() else "end"


def publish_denm(
	vehicle: VehicleSim,
	cause_code: int = DENM_CAUSE_COLLISION_RISK,
	sub_cause_code: int = 0,
	validity_duration: int = 10,
	event_position: Optional[Tuple[float, float]] = None,
	notify_vehicles: Optional[List[VehicleSim]] = None,
) -> None:
	"""Publish a DENM through Vanetza and directly to peer brokers.

	Two-path strategy:
	  1. vanetza/in/denm  — correct ITS-G5 path: Vanetza encodes and broadcasts.
	  2. vanetza/out/denm on each peer broker — direct fallback so the webapp
	     always visualises the alert even if Vanetza-NAP does not fully support
	     DENM injection. Also mirrors what the receiving OBU would publish after
	     decoding the over-the-air packet.

	notify_vehicles: list of other VehicleSim instances that should receive the
	alert directly on their broker's vanetza/out/denm. Pass all vehicles in the
	scenario except the originator.
	"""
	its_epoch_offset = 1072915200

	def timestamp_its() -> int:
		return int((time.time() - its_epoch_offset) * 1000)

	# ── Vanetza input format (management + situation only) ──────────────
	denm_in_payload = {
		"management": {
			"actionId": {
				"originatingStationId": vehicle.station_id,
				"sequenceNumber": 1,
			},
			"detectionTime": time.time(),
			"referenceTime": time.time(),
			"eventPosition": {
				"latitude": event_position[0] if event_position else vehicle.current_lat,
				"longitude": event_position[1] if event_position else vehicle.current_lon,
				"positionConfidenceEllipse": {
					"semiMajorConfidence": 50,
					"semiMinorConfidence": 50,
					"semiMajorOrientation": 0,
				},
				"altitude": {
					"altitudeValue": 0,
					"altitudeConfidence": 1,
				},
			},
			"stationType": 0,
			"validityDuration": validity_duration,
		},
		"situation": {
			"informationQuality": 7,
			"eventType": {
				"ccAndScc": {
					"wrongWayDriving14": 0,
				},
			},
		},
	}

	# ── Vanetza output format (what the backend/webapp expects) ─────────
	denm_out_payload = {
		"timestamp": time.time(),
		"rssi": -16,
		"stationID": vehicle.station_id,
		"newInfo": True,
		"fields": {
			"header": {
				"protocolVersion": 2,
				"messageId": 1,
				"stationId": vehicle.station_id,
			},
			"denm": denm_in_payload,
		},
	}

	# Path 1: proper ITS-G5 injection
	vehicle.client.publish(DENM_TOPIC_IN, json.dumps(denm_in_payload), qos=0)

	# Note: direct publish to peer brokers has been removed to enforce
	# correct ITS-G5 flow. DENMs are published only via Vanetza on
	# `vanetza/in/denm` so the codec/broker handles distribution.

	print(
		f"[DENM] Published by {vehicle.name} "
		f"causeCode={cause_code} subCauseCode={sub_cause_code} "
		f"at ({(event_position[0] if event_position else vehicle.current_lat):.6f}, "
		f"{(event_position[1] if event_position else vehicle.current_lon):.6f})"
	)
