"""MVP V2X simulator with OSRM routing (real roads).

This script emulates vehicles moving through real roads using OSRM routing
and publishes CAM messages periodically to each OBU MQTT broker.
"""

from __future__ import annotations

import json
import math
import signal
import time
import requests
from dataclasses import dataclass
from typing import List, Tuple

import paho.mqtt.client as mqtt


CAM_TOPIC_IN = "vanetza/in/cam"
TICK_HZ = 5.0
TICK_SECONDS = 1.0 / TICK_HZ

# OSRM server (usar público - sem setup!)
OSRM_SERVER = "http://router.project-osrm.org"
WARNING_DISTANCE_M = 80.0
YIELD_DISTANCE_M = 35.0
REVERSE_DISTANCE_M = 20.0
CLEAR_DISTANCE_M = 28.0


def haversine_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
	"""Compute distance between 2 WGS84 coordinates in meters."""
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


def bearing_degrees(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
	"""Compute heading from point A to point B in degrees [0, 360)."""
	phi1 = math.radians(lat1)
	phi2 = math.radians(lat2)
	dlambda = math.radians(lon2 - lon1)

	x = math.sin(dlambda) * math.cos(phi2)
	y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(
		dlambda
	)

	brng = math.degrees(math.atan2(x, y))
	return (brng + 360.0) % 360.0

###
def interpolate(
	lat1: float, lon1: float, lat2: float, lon2: float, ratio: float
) -> Tuple[float, float]:
	"""Linear interpolation for short segments in local city scale."""
	return (lat1 + (lat2 - lat1) * ratio, lon1 + (lon2 - lon1) * ratio)


def route_length_m(route: List[Tuple[float, float]]) -> float:
	"""Sum the total length of a route in meters."""
	return sum(haversine_meters(*route[i], *route[i + 1]) for i in range(len(route) - 1))


def heading_delta_degrees(a_deg: float, b_deg: float) -> float:
	"""Return the smallest absolute difference between two headings."""
	return abs(((a_deg - b_deg + 180.0) % 360.0) - 180.0)

# Preencher o campo obrigatório das mensagens CAM com um valor que varia ao longo do tempo, como a geraçãoDeltaTime, para garantir que cada mensagem seja única e possa ser processada corretamente pelos receptores.
def generation_delta_time() -> int:
	"""ETSI generationDeltaTime in milliseconds modulo 65536."""
	return int((time.time() * 1000.0) % 65536)


def get_osrm_route(start_lat: float, start_lon: float, end_lat: float, end_lon: float) -> List[Tuple[float, float]]:
	"""Pede rota ao OSRM e retorna lista de waypoints nas estradas reais."""
	try:
		# Format: /route/v1/profile/coordinates
		url = f"{OSRM_SERVER}/route/v1/driving/{start_lon},{start_lat};{end_lon},{end_lat}"
		params = {
			"overview": "full",
			"geometries": "geojson",
			"steps": "false"
		}
		
		response = requests.get(url, params=params, timeout=10)
		response.raise_for_status()
		
		data = response.json()
		
		if data.get("code") != "Ok":
			print(f"⚠️ OSRM error: {data.get('message')}")
			# Fallback: reta simples
			return [(start_lat, start_lon), (end_lat, end_lon)]
		
		# Extrair waypoints da rota
		route = data.get("routes", [{}])[0]
		geometry = route.get("geometry", {})
		coordinates = geometry.get("coordinates", [])
		
		# Converter de [lon, lat] para (lat, lon)
		waypoints = [(lat, lon) for lon, lat in coordinates]
		
		if waypoints:
			print(f"✅ Rota obtida: {len(waypoints)} waypoints")
			return waypoints
		else:
			return [(start_lat, start_lon), (end_lat, end_lon)]
			
	except Exception as e:
		print(f"⚠️ OSRM request failed: {e}")
		# Fallback: reta simples
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
	start_point: Tuple[float, float]  # (lat, lon)
	end_point: Tuple[float, float]    # (lat, lon)
	base_speed_mps: float
	collision_count: int = 0  # Contador de colisões
	total_route_length_m: float = 0.0

	def __post_init__(self) -> None:
		###
		self.client = mqtt.Client(client_id=f"sim-{self.station_id}")
		self.client.connect(self.broker_host, 1883, 60)
		self.client.loop_start()

		# Pedir rota ao OSRM (estradas reais!)
		print(f"[{self.name}] 🗺️ Pedindo rota OSRM...")
		self.route = get_osrm_route(
			self.start_point[0], self.start_point[1],
			self.end_point[0], self.end_point[1]
		)
		
		if len(self.route) < 2:
			print(f"⚠️ {self.name}: rota inválida!")
			self.route = [self.start_point, self.end_point]

		self.total_route_length_m = route_length_m(self.route)

		self.segment_idx = 0
		self.current_lat, self.current_lon = self.route[0]
		self.last_heading_deg = bearing_degrees(*self.route[0], *self.route[1])

		self.current_speed_mps = self.base_speed_mps
		self.target_speed_mps = self.base_speed_mps
		self.in_collision_avoidance = False  # Flag para modo evasão

	def distance_from_route_start_m(self) -> float:
		"""Distance traveled from the first waypoint to the current position."""
		distance = 0.0
		for idx in range(self.segment_idx):
			distance += haversine_meters(*self.route[idx], *self.route[idx + 1])
		distance += haversine_meters(*self.route[self.segment_idx], self.current_lat, self.current_lon)
		return distance

	def distance_to_route_end_m(self) -> float:
		"""Distance from the current position to the last waypoint."""
		return max(self.total_route_length_m - self.distance_from_route_start_m(), 0.0)

	###
	def step_and_publish(self, dt: float) -> None:
		# defenir rota (p1 -> p2) e calcular a distancia da rota
		p1 = self.route[self.segment_idx]
		p2 = self.route[(self.segment_idx + 1) % len(self.route)]
		seg_dist = max(haversine_meters(*p1, *p2), 0.01)
		# distancia percorrida por tick
		self.current_speed_mps += (self.target_speed_mps - self.current_speed_mps) * 0.15
		move_dist = self.current_speed_mps * dt

		# saber se ainda está no inicio ou já avançou suficiente para estar perto do fim
		dist_from_p1 = haversine_meters(*p1, self.current_lat, self.current_lon)
		progress = min(max(dist_from_p1 / seg_dist, 0.0), 1.0)
		# quanto avançou em relação ao segmento atual
		step_ratio = move_dist / seg_dist
		# calcula posiçao relativa, do proximo tick
		next_progress = progress + step_ratio

		if move_dist >= 0.0:
			while next_progress >= 1.0:
				# avança pro proximo segmento da rota
				self.segment_idx = (self.segment_idx + 1) % len(self.route)
				# atualiza pontos e distancia do segmento
				p1 = self.route[self.segment_idx]
				p2 = self.route[(self.segment_idx + 1) % len(self.route)]
				seg_dist = max(haversine_meters(*p1, *p2), 0.01)
				# remove progress, pq começou noutro segmento
				next_progress -= 1.0

			# posição exata no segmento atual
			self.current_lat, self.current_lon = interpolate(*p1, *p2, next_progress)
			# direçao do movimento
			self.last_heading_deg = bearing_degrees(*p1, *p2)
		else:
			while next_progress < 0.0:
				if self.segment_idx == 0:
					next_progress = 0.0
					self.current_speed_mps = 0.0
					self.target_speed_mps = 0.0
					break

				self.segment_idx -= 1
				p1 = self.route[self.segment_idx]
				p2 = self.route[self.segment_idx + 1]
				seg_dist = max(haversine_meters(*p1, *p2), 0.01)
				next_progress += 1.0

			# posição exata no segmento atual, em marcha-atrás
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


RUNNING = True
LAST_DENM_TIME = 0.0  # Cooldown para DENMs


def signal_handler(_signum: int, _frame: object) -> None:
	global RUNNING
	RUNNING = False


def detect_collision_risk(v1: VehicleSim, v2: VehicleSim) -> bool:
    """Deteta se a distância entre dois veículos é inferior a 20 metros."""
    distance = haversine_meters(
        v1.current_lat, v1.current_lon,
        v2.current_lat, v2.current_lon
    )
	
    if distance < 20.0:
        print(f"🚨 RISK DETECTED! DIST={distance:.2f}m")
        return True
    return False


def vehicles_share_same_road_and_opposite_direction(v1: VehicleSim, v2: VehicleSim) -> bool:
	"""Heuristic to detect vehicles on the same street, facing each other."""
	start_to_end_1 = bearing_degrees(*v1.start_point, *v1.end_point)
	start_to_end_2 = bearing_degrees(*v2.start_point, *v2.end_point)
	heading_gap = heading_delta_degrees(start_to_end_1, start_to_end_2)
	shared_endpoint_gap = min(
		haversine_meters(*v1.end_point, *v2.start_point),
		haversine_meters(*v1.start_point, *v2.end_point),
	)
	return heading_gap > 150.0 and shared_endpoint_gap < 35.0


def choose_yield_vehicle(v1: VehicleSim, v2: VehicleSim) -> VehicleSim:
	"""Choose the vehicle that should yield first.

	Prefer the vehicle that can clear the road with the smallest maneuver.
	If one is near the start and the other near the end, the one near the start
	backs out and the one near the end keeps moving forward.
	"""
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
	"""Return True only when both vehicles are actually closing in on each other."""
	bearing_1_to_2 = bearing_degrees(v1.current_lat, v1.current_lon, v2.current_lat, v2.current_lon)
	bearing_2_to_1 = bearing_degrees(v2.current_lat, v2.current_lon, v1.current_lat, v1.current_lon)
	heading_gap_1 = heading_delta_degrees(v1.last_heading_deg, bearing_1_to_2)
	heading_gap_2 = heading_delta_degrees(v2.last_heading_deg, bearing_2_to_1)
	return heading_gap_1 < 70.0 and heading_gap_2 < 70.0


def vehicle_nearest_exit(vehicle: VehicleSim) -> str:
	"""Return whether the vehicle is closer to the route start or end."""
	return "start" if vehicle.distance_from_route_start_m() <= vehicle.distance_to_route_end_m() else "end"

def publish_denm(vehicle: VehicleSim):

    ITS_EPOCH_OFFSET = 1072915200

    def timestamp_its():
        return int((time.time() - ITS_EPOCH_OFFSET) * 1000)

    # Estrutura DENM conforme output de Vanetza (com header e fields)
    denm_payload = {
        "timestamp": time.time(),
        "rssi": -16,
        "stationID": vehicle.station_id,
        "stationAddr": f"6e:06:e0:01:00:{vehicle.station_id:02x}",
        "receiverID": 229,
        "receiverType": 5,
        "packet_size": 103,
        "fields": {
            "header": {
                "protocolVersion": 2,
                "messageId": 1,
                "stationId": vehicle.station_id
            },
            "denm": {
                "management": {
                    "actionId": {
                        "originatingStationId": vehicle.station_id,
                        "sequenceNumber": 1
                    },
                    "detectionTime": timestamp_its(),
                    "referenceTime": timestamp_its(),
                    "eventPosition": {
                        "latitude": vehicle.current_lat,
                        "longitude": vehicle.current_lon,
                        "positionConfidenceEllipse": {
                            "semiMajorConfidence": 50,
                            "semiMinorConfidence": 50,
                            "semiMajorOrientation": 0.0
                        },
                        "altitude": {
                            "altitudeValue": 0.0,
                            "altitudeConfidence": 1
                        }
                    },
                    "stationType": 5,
                    "validityDuration": 10
                },
                "situation": {
                    "informationQuality": 7,
                    "eventType": {
                        "ccAndScc": {
                            "proximityAlert1": 1
                        }
                    }
                }
            }
        }
    }

    # Publica directamente em vanetza/out/denm (bypass Vanetza container)
    vehicle.client.publish(
        "vanetza/out/denm",
        json.dumps(denm_payload),
        qos=0
    )

    print(f"[DENM] Published by {vehicle.name} at ({vehicle.current_lat:.6f}, {vehicle.current_lon:.6f})")

def main() -> None:
	signal.signal(signal.SIGINT, signal_handler)
	signal.signal(signal.SIGTERM, signal_handler)

	# ═══════════════════════════════════════════════════════════════
	# CENÁRIO REALISTA: Rua de 1 via, 2 sentidos
	# ═══════════════════════════════════════════════════════════════
	# 
	# Visualização:
	#
	#  OBU1 →  |  ← OBU2
	#     entrada    entrada
	#         |
	#      (rua curta com 1 via)
	#         |
	#       saída
	#
	# Resultado: Colisão iminente!
	# Reação: Um dos carros espera ou sai
	# ═══════════════════════════════════════════════════════════════
	
	print("\n" + "="*60)
	print("🚗 V2X SIMULATOR - CENÁRIO REALISTA")
	print("="*60)
	print("📍 Localização: R. Arnaçó, Braga")
	print("🛣️  Rua estreita com veículos em sentidos opostos")
	print("⚠️  Aviso cedo para parar, encostar ou recuar")
	print("="*60 + "\n")
	
	# Coordenadas reais de Braga
	# R. Arnaçó: cenário curto para reagir cedo
	
	vehicles = [
		VehicleSim(
			name="obu1",
			station_id=2,
			broker_host="192.168.98.20",
			# OBU1: R. Arnaçó 40 -> R. Arnaçó 50
			start_point=(41.727849, -8.163264),
			end_point=(41.725762, -8.165449),
			base_speed_mps=8.0,
		),
		VehicleSim(
			name="obu2",
			station_id=3,
			broker_host="192.168.98.21",
			# OBU2: R. Arnaçó 50 -> R. Arnaçó 2
			start_point=(41.725639, -8.165512),
			end_point=(41.727598, -8.163408),
			base_speed_mps=9.0,
		),
	]
	
	print("🚙 OBU1 (azul):  40 -> 50")
	print("🚙 OBU2 (laranja): 50 -> 2")
	print("\n⏱️  Aviso deve surgir antes dos 20m")
	print("📡 Esperando DENM e reação de cedência...\n")

	###
	tick_count = 0
	collision_detection_time = 0.0
	avoidance_active = False
	yield_vehicle_name = ""
	yield_mode = "stop"
	
	while RUNNING:
		tick_start = time.time()
		global LAST_DENM_TIME

		# 1. Passo de movimento para todos
		for vehicle in vehicles:
			vehicle.step_and_publish(TICK_SECONDS)

		# 2. Lógica de risco: mesma rua e sentidos opostos
		distance = haversine_meters(
			vehicles[0].current_lat, vehicles[0].current_lon,
			vehicles[1].current_lat, vehicles[1].current_lon
		)
		same_road_opposite = vehicles_share_same_road_and_opposite_direction(vehicles[0], vehicles[1])
		approaching_each_other = vehicles_are_approaching_each_other(vehicles[0], vehicles[1])
		now = time.time()
		
		if same_road_opposite and approaching_each_other and distance < WARNING_DISTANCE_M:
			if not avoidance_active and now - LAST_DENM_TIME > 5.0:
				yield_vehicle = choose_yield_vehicle(vehicles[0], vehicles[1])
				priority_vehicle = vehicles[1] if yield_vehicle is vehicles[0] else vehicles[0]
				yield_side = vehicle_nearest_exit(yield_vehicle)
				yield_distance = (
					yield_vehicle.distance_from_route_start_m()
					if yield_side == "start"
					else yield_vehicle.distance_to_route_end_m()
				)
				yield_mode = "reverse" if yield_distance <= REVERSE_DISTANCE_M else "stop"
				yield_vehicle_name = yield_vehicle.name
				avoidance_active = True
				collision_detection_time = now

				print(f"\n⚠️ AVISO CEDO: veículos na mesma rua e em sentidos opostos")
				print(f"   Distância atual: {distance:.2f}m")
				print(
					f"   {yield_vehicle.name} deve ceder ({yield_mode}); "
					f"{priority_vehicle.name} mantém prioridade"
				)
				print(
					f"   {yield_vehicle.name} está mais perto do {yield_side} da rota; "
					f"margem livre: {yield_distance:.1f}m"
				)

				vehicles[0].in_collision_avoidance = True
				vehicles[1].in_collision_avoidance = True
				print(f"📡 [DENM] Aviso publicado para ambos os veículos")
				publish_denm(vehicles[0])
				publish_denm(vehicles[1])
				LAST_DENM_TIME = now
			for vehicle in vehicles:
				if vehicle.name == yield_vehicle_name:
					if yield_mode == "reverse":
						# Se ainda está perto da entrada, recua; depois fica parado até abrir espaço.
						if distance > YIELD_DISTANCE_M:
							vehicle.target_speed_mps = -min(2.5, max(1.2, vehicle.base_speed_mps * 0.35))
						else:
							vehicle.target_speed_mps = 0.0
					else:
						# Se estiver a meio da rua, abranda forte; perto do conflito, pára.
						if distance > YIELD_DISTANCE_M:
							vehicle.target_speed_mps = vehicle.base_speed_mps * 0.25
						else:
							vehicle.target_speed_mps = 0.0
				else:
					# O outro avança com cuidado, mas sem ficar demasiado lento.
					vehicle.target_speed_mps = vehicle.base_speed_mps * (0.80 if distance > YIELD_DISTANCE_M else 0.60)
		else:
			if avoidance_active and distance > CLEAR_DISTANCE_M and not approaching_each_other:
				print(f"✅ veículos já passaram um pelo outro; arranque gradual liberado")
				avoidance_active = False
				yield_vehicle_name = ""
				yield_mode = "stop"
				for vehicle in vehicles:
					vehicle.in_collision_avoidance = False

			# Sem conflito ativo: cada veículo volta em rampa suave para a velocidade base.
			for vehicle in vehicles:
				if not vehicle.in_collision_avoidance:
					vehicle.target_speed_mps = vehicle.base_speed_mps

		tick_count += 1
		if tick_count % int(TICK_HZ) == 0:
			for vehicle in vehicles:
				print(
					(
						f"[{vehicle.name}] lat={vehicle.current_lat:.6f} "
						f"lon={vehicle.current_lon:.6f} speed={vehicle.current_speed_mps:.1f}m/s "
						f"heading={vehicle.last_heading_deg:.1f}deg"
					)
				)

		elapsed = time.time() - tick_start
		sleep_time = max(0.0, TICK_SECONDS - elapsed)
		time.sleep(sleep_time)

	for vehicle in vehicles:
		vehicle.close()

	print("Simulator stopped")


if __name__ == "__main__":
	main()
