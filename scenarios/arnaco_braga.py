"""Scenario: R. Arnaçó, Braga."""

from __future__ import annotations

import signal
import time

from simulator_core import (
	CLEAR_DISTANCE_M,
	DENM_CAUSE_COLLISION_RISK,
	LAST_DENM_TIME,
	REVERSE_DISTANCE_M,
	TICK_HZ,
	TICK_SECONDS,
	WARNING_DISTANCE_M,
	YIELD_DISTANCE_M,
	VehicleSim,
	choose_yield_vehicle,
	publish_denm,
	vehicle_nearest_exit,
	vehicles_are_approaching_each_other,
	vehicles_share_same_road_and_opposite_direction,
	haversine_meters,
)

RUNNING = True


def signal_handler(_signum: int, _frame: object) -> None:
	global RUNNING
	RUNNING = False


def run() -> None:
	global LAST_DENM_TIME

	signal.signal(signal.SIGINT, signal_handler)
	signal.signal(signal.SIGTERM, signal_handler)

	print("\n" + "=" * 60)
	print("🚗 V2X SIMULATOR - CENÁRIO: R. Arnaçó, Braga")
	print("=" * 60)
	print("📍 Rua estreita com veículos em sentidos opostos")
	print("⚠️  Aviso cedo para parar, encostar ou recuar")
	print("=" * 60 + "\n")

	vehicles = [
		VehicleSim(
			name="obu1",
			station_id=2,
			broker_host="192.168.98.20",
			start_point=(41.727849, -8.163264),
			end_point=(41.725762, -8.165449),
			base_speed_mps=8.0,
		),
		VehicleSim(
			name="obu2",
			station_id=3,
			broker_host="192.168.98.21",
			start_point=(41.725639, -8.165512),
			end_point=(41.727598, -8.163408),
			base_speed_mps=9.0,
		),
	]

	print("🚙 OBU1 (azul):  40 -> 50")
	print("🚙 OBU2 (laranja): 50 -> 2")
	print("\n⏱️  Aviso deve surgir antes dos 20m")
	print("📡 Esperando DENM e reação de cedência...\n")

	tick_count = 0
	collision_detection_time = 0.0
	avoidance_active = False
	yield_vehicle_name = ""
	yield_mode = "stop"
	yield_resume_time = 0.0

	while RUNNING:
		tick_start = time.time()

		for vehicle in vehicles:
			vehicle.step_and_publish(TICK_SECONDS)

		distance = haversine_meters(
			vehicles[0].current_lat, vehicles[0].current_lon,
			vehicles[1].current_lat, vehicles[1].current_lon,
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
				yield_resume_time = 0.0
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
				print("📡 [DENM] Aviso publicado para ambos os veículos")
				publish_denm(vehicles[0], cause_code=DENM_CAUSE_COLLISION_RISK, notify_vehicles=[vehicles[1]])
				publish_denm(vehicles[1], cause_code=DENM_CAUSE_COLLISION_RISK, notify_vehicles=[vehicles[0]])
				LAST_DENM_TIME = now

			for vehicle in vehicles:
				if vehicle.name == yield_vehicle_name:
					if yield_mode == "reverse":
						if distance > YIELD_DISTANCE_M:
							vehicle.target_speed_mps = -min(2.5, max(1.2, vehicle.base_speed_mps * 0.35))
						else:
							vehicle.target_speed_mps = 0.0
					else:
						if distance > YIELD_DISTANCE_M:
							vehicle.target_speed_mps = vehicle.base_speed_mps * 0.25
						else:
							vehicle.target_speed_mps = 0.0
				else:
					vehicle.target_speed_mps = vehicle.base_speed_mps * (0.80 if distance > YIELD_DISTANCE_M else 0.60)
		else:
			if avoidance_active and distance > CLEAR_DISTANCE_M and not approaching_each_other:
				print("✅ veículos já passaram um pelo outro; arranque gradual liberado")
				avoidance_active = False
				yield_vehicle_name = ""
				yield_mode = "stop"
				yield_resume_time = now + 2.0
				for vehicle in vehicles:
					vehicle.in_collision_avoidance = False

			for vehicle in vehicles:
				if not vehicle.in_collision_avoidance:
					if now < yield_resume_time:
						vehicle.target_speed_mps = vehicle.base_speed_mps * 0.35
					else:
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
		time.sleep(max(0.0, TICK_SECONDS - elapsed))

	for vehicle in vehicles:
		vehicle.close()

	print("Simulator stopped")