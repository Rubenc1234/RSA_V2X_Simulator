"""MVP V2X simulator focused on OBU containers.

This script emulates two vehicles moving through predefined GPS routes and
publishes CAM messages periodically to each OBU MQTT broker.
"""

from __future__ import annotations

import json
import math
import signal
import time
from dataclasses import dataclass
from typing import List, Tuple

import paho.mqtt.client as mqtt


CAM_TOPIC_IN = "vanetza/in/cam"
TICK_HZ = 5.0
TICK_SECONDS = 1.0 / TICK_HZ

DENM_TOPIC_IN = "vanetza/in/denm"
LAST_DENM_TIME = 0.0  # Para o cooldown global


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

# Preencher o campo obrigatório das mensagens CAM com um valor que varia ao longo do tempo, como a geraçãoDeltaTime, para garantir que cada mensagem seja única e possa ser processada corretamente pelos receptores.
def generation_delta_time() -> int:
	"""ETSI generationDeltaTime in milliseconds modulo 65536."""
	return int((time.time() * 1000.0) % 65536)


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
	route: List[Tuple[float, float]]
	base_speed_mps: float

	def __post_init__(self) -> None:
		if len(self.route) < 2:
			raise ValueError(f"Vehicle {self.name} route needs at least 2 points")

		###
		self.client = mqtt.Client(client_id=f"sim-{self.station_id}")
		self.client.connect(self.broker_host, 1883, 60)
		self.client.loop_start()

		self.segment_idx = 0
		self.current_lat, self.current_lon = self.route[0]
		self.last_heading_deg = bearing_degrees(*self.route[0], *self.route[1])

		self.current_speed_mps = self.base_speed_mps
		self.target_speed_mps = self.base_speed_mps

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

	# Rota OBU1: De Oeste para Leste (esquerda para direita) - com zigzag interessante
	obu1_route = [
        (40.62840, -8.65600),  # Oeste
        (40.62850, -8.65500),  # Curva ligeira
        (40.62845, -8.65400),  # Centro do cruzamento
        (40.62835, -8.65300),  # Curva ligeira
        (40.62840, -8.65200),  # Leste
    ]

    # Rota OBU2: De Norte para Sul (cima para baixo) - cruzamento perpendicular
	obu2_route = [
        (40.62900, -8.65350),  # Norte
        (40.62870, -8.65350),  # Curva ligeira
        (40.62845, -8.65350),  # Centro do cruzamento (AQUI CRUZA OBU1!)
        (40.62820, -8.65350),  # Curva ligeira
        (40.62790, -8.65350),  # Sul
    ]

	# info dos veiculos, dada no docker
	vehicles = [
		VehicleSim(
			name="obu1",
			station_id=2,
			broker_host="192.168.98.20",
			route=obu1_route,
			base_speed_mps=10.0,
		),
		VehicleSim(
			name="obu2",
			station_id=3,
			broker_host="192.168.98.21",
			route=obu2_route,
			base_speed_mps=12.0,
		),
	]

	print("Simulator started: publishing CAM at 5 Hz for 2 OBUs")
	print("Press Ctrl+C to stop")

	###
	tick_count = 0
	while RUNNING:
		tick_start = time.time()
		global LAST_DENM_TIME

        # 1. Passo de movimento para todos
		for vehicle in vehicles:
			vehicle.step_and_publish(TICK_SECONDS)

        # 2. Lógica de Risco (Simples: OBU1 vs OBU2)
		if detect_collision_risk(vehicles[0], vehicles[1]):
            # Reação automática imediata
			for v in vehicles:
				v.target_speed_mps = 0.0 # Emergency Stop
            
            # Publicação de DENM com Cooldown de 5 segundos
			if (time.time() - LAST_DENM_TIME) > 5.0:
				print(f"Publishing DENM due to detected risk (Cooldown OK)")
				publish_denm(vehicles[0])
				LAST_DENM_TIME = time.time()
		else:
            # Se não há risco, tentam voltar à velocidade base
			for v in vehicles:
				v.target_speed_mps = v.base_speed_mps

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
