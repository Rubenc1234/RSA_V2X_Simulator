"""Scenario: RSU-managed intersection with 4 OBUs.

Four vehicles approach an intersection from different directions.
The RSU monitors their positions via CAMs and issues a DENM when two or more
vehicles are within the warning radius at the same time, granting priority to
one and holding the others until the intersection clears.

All routes use verified on-road GPS waypoints.

Intersection location: ~41°11'24"N 8°37'59"W (Porto area)

Vehicle movements:
  OBU1 — east → left turn → south
  OBU2 — west → left turn → north
  OBU3 — north → left turn → east
  OBU4 — south → straight through → north  (no turn)

Cause codes:
  97 = intersection management (RSU controlling access)
  26 = collisionRisk (cleared / resume signal)

⚠️  OBU4 requires a new entry in vanetza-nap/docker-compose.yml:
      name: obu4
      IP:   192.168.98.23
      VANETZA_STATION_ID:   5
      VANETZA_STATION_TYPE: 5  (passengerCar)
      MAC:  6e:06:e0:03:00:05
    Copy the obu3 service block, adjust the fields above, then:
      cd vanetza-nap && docker-compose up -d
"""

from __future__ import annotations

import signal
import time
from typing import List

from simulator_core import (
    DENM_CAUSE_COLLISION_RISK,
    TICK_HZ,
    TICK_SECONDS,
    RsuSim,
    VehicleSim,
    haversine_meters,
    route_length_m,
)

DENM_CAUSE_INTERSECTION = 97

INTERSECTION_WARNING_M = 50.0   # vehicles inside this radius trigger conflict detection
INTERSECTION_STOP_M    = 18.0   # non-priority vehicles halt at this distance
INTERSECTION_CLEAR_M   = 32.0   # priority vehicle is "clear" once past this radius

RUNNING = True
RSU_CAM_INTERVAL = 1.0

# ── Reference points ──────────────────────────────────────────────────────────
RSU_POSITION        = (41.190028, -8.632889)   # off-road SE corner — 41°11'24.1"N 8°37'58.4"W
INTERSECTION_CENTRE = (41.190100, -8.633000)   # road-centre used for distance calculations

# ── Route waypoints ───────────────────────────────────────────────────────────

# OBU1: approaching from east (heading west), left turn → exits southbound
OBU1_ROUTE = [
    (41.190611, -8.630694),   # 41°11'26.2"N 8°37'50.5"W  start
    (41.190361, -8.631389),   # 41°11'25.3"N 8°37'53.0"W
    (41.190278, -8.632250),   # 41°11'25.0"N 8°37'56.1"W
    (41.190167, -8.632944),   # 41°11'24.6"N 8°37'58.6"W  ← stop for intersection
    (41.190028, -8.633000),   # 41°11'24.1"N 8°37'58.8"W
    (41.189806, -8.632944),   # 41°11'23.3"N 8°37'58.6"W
    (41.189528, -8.632861),   # 41°11'22.3"N 8°37'58.3"W  end
]

# OBU2: approaching from west (heading east), left turn → exits northbound
OBU2_ROUTE = [
    (41.189972, -8.635250),   # 41°11'23.9"N 8°38'06.9"W  start
    (41.189917, -8.634417),   # 41°11'23.7"N 8°38'03.9"W
    (41.189972, -8.633694),   # 41°11'23.9"N 8°38'01.3"W
    (41.190056, -8.633028),   # 41°11'24.2"N 8°37'58.9"W  ← stop for intersection
    (41.190222, -8.633000),   # 41°11'24.8"N 8°37'58.8"W
    (41.191278, -8.633306),   # 41°11'28.6"N 8°37'59.9"W  end
]

# OBU3: approaching from north (heading south), left turn → exits eastbound
OBU3_ROUTE = [
    (41.191361, -8.633361),   # 41°11'28.9"N 8°38'00.1"W  start
    (41.190806, -8.633194),   # 41°11'26.9"N 8°37'59.5"W
    (41.190167, -8.633028),   # 41°11'24.6"N 8°37'58.9"W  ← stop for intersection
    (41.190083, -8.632944),   # 41°11'24.3"N 8°37'58.6"W
    (41.190111, -8.632750),   # 41°11'24.4"N 8°37'57.9"W
    (41.190250, -8.631722),   # 41°11'24.9"N 8°37'54.2"W  end
]

# OBU4: approaching from south (heading north), straight through → exits northbound
OBU4_ROUTE = [
    (41.188833, -8.632639),   # 41°11'19.8"N 8°37'57.5"W  start
    (41.189472, -8.632833),   # 41°11'22.1"N 8°37'58.2"W
    (41.190056, -8.632972),   # 41°11'24.2"N 8°37'58.7"W  ← stop for intersection
    (41.190389, -8.633056),   # 41°11'25.4"N 8°37'59.0"W
    (41.191306, -8.633306),   # 41°11'28.7"N 8°37'59.9"W  end
]


def signal_handler(_signum: int, _frame: object) -> None:
    global RUNNING
    RUNNING = False


def vehicles_near_intersection(vehicles: List[VehicleSim], radius_m: float) -> List[VehicleSim]:
    return [
        v for v in vehicles
        if haversine_meters(v.current_lat, v.current_lon, *INTERSECTION_CENTRE) < radius_m
    ]


def choose_priority_vehicle(near: List[VehicleSim]) -> VehicleSim:
    """Grant priority to the vehicle with the least remaining route distance.

    The vehicle closest to clearing the intersection proceeds first;
    holding it longer only extends the congestion window for the others.
    """
    return min(near, key=lambda v: v.distance_to_route_end_m())


def run() -> None:
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("\n" + "=" * 60)
    print(" V2X SIMULATOR — RSU INTERSECTION SCENARIO (4 OBUs)")
    print("=" * 60)
    print(" Porto area — verified GPS routes, 5 m/s")
    print(" OBU1: east  → left turn  → south")
    print(" OBU2: west  → left turn  → north")
    print(" OBU3: north → left turn  → east")
    print(" OBU4: south → straight   → north")
    print("  RSU detects conflict and grants priority via DENM (code 97)")
    print("=" * 60 + "\n")

    rsu = RsuSim(
        name="rsu",
        station_id=1,
        broker_host="192.168.98.10",
        position=RSU_POSITION,
    )

    vehicles = [
        VehicleSim(
            name="obu1",
            station_id=2,
            broker_host="192.168.98.20",
            start_point=OBU1_ROUTE[0],
            end_point=OBU1_ROUTE[-1],
            base_speed_mps=6.0,
            manual_route=OBU1_ROUTE,
        ),
        VehicleSim(
            name="obu2",
            station_id=3,
            broker_host="192.168.98.21",
            start_point=OBU2_ROUTE[0],
            end_point=OBU2_ROUTE[-1],
            base_speed_mps=6.0,
            manual_route=OBU2_ROUTE,
        ),
        VehicleSim(
            name="obu3",
            station_id=4,
            broker_host="192.168.98.22",
            start_point=OBU3_ROUTE[0],
            end_point=OBU3_ROUTE[-1],
            base_speed_mps=6.0,
            manual_route=OBU3_ROUTE,
        ),
        VehicleSim(
            name="obu4",
            station_id=5,
            broker_host="192.168.98.23",
            start_point=OBU4_ROUTE[0],
            end_point=OBU4_ROUTE[-1],
            base_speed_mps=6.0,
            manual_route=OBU4_ROUTE,
        ),
    ]

    rsu.subscribe_to_broker("192.168.98.20", "obu1")
    rsu.subscribe_to_broker("192.168.98.21", "obu2")
    rsu.subscribe_to_broker("192.168.98.22", "obu3")
    rsu.subscribe_to_broker("192.168.98.23", "obu4")

    print(f" Intersection centre (detection ref): {INTERSECTION_CENTRE}")
    print(f" RSU position (off-road):             {RSU_POSITION}")
    for v in vehicles:
        length = round(route_length_m(v.route))
        print(f" {v.name}: {len(v.route)} waypoints, {length}m @ {v.base_speed_mps}m/s")
    print()

    state: str = "NORMAL"
    priority_vehicle_name: str = ""
    last_denm_time: float = 0.0
    last_rsu_cam_time: float = 0.0
    denm_interval_s: float = 3.0
    clear_confirm_start: float = 0.0
    priority_in_intersection: bool = False
    # Vehicles that have already passed through — excluded from priority
    # selection and speed control so they don't block subsequent cycles.
    cleared_vehicles: set = set()

    tick_count = 0
    while RUNNING:
        tick_start = time.time()
        now = time.time()

        for vehicle in vehicles:
            vehicle.step_and_publish(TICK_SECONDS)

        # RSU broadcasts its own CAM at 1 Hz so the webapp can show its icon
        if now - last_rsu_cam_time >= RSU_CAM_INTERVAL:
            rsu.publish_cam()
            last_rsu_cam_time = now

        near = vehicles_near_intersection(vehicles, INTERSECTION_WARNING_M)
        dist_to_intersection = {
            v.name: haversine_meters(v.current_lat, v.current_lon, *INTERSECTION_CENTRE)
            for v in vehicles
        }

        # ── NORMAL ───────────────────────────────────────────────────────────
        if state == "NORMAL":
            # Only vehicles that haven't already crossed count as conflicting
            conflict_near = [v for v in near if v.name not in cleared_vehicles]
            if len(conflict_near) >= 2:
                priority_vehicle = choose_priority_vehicle(conflict_near)
                priority_vehicle_name = priority_vehicle.name
                state = "MANAGED"
                last_denm_time = 0.0
                priority_in_intersection = False
                clear_confirm_start = 0.0
                print(
                    f"\n🚦 CONFLICT DETECTED: {[v.name for v in conflict_near]} within "
                    f"{INTERSECTION_WARNING_M}m"
                )
                print(f"   Priority granted → {priority_vehicle_name}")

        # ── MANAGED ──────────────────────────────────────────────────────────
        if state == "MANAGED":
            priority_vehicle = next(
                (v for v in vehicles if v.name == priority_vehicle_name), None
            )

            # Re-broadcast DENM every denm_interval_s seconds
            if now - last_denm_time > denm_interval_s:
                rsu.publish_denm(
                    cause_code=DENM_CAUSE_INTERSECTION,
                    sub_cause_code=0,
                    validity_duration=denm_interval_s + 2,
                    notify_vehicles=vehicles,
                )
                last_denm_time = now

            # Speed control: priority runs freely; non-priority slow/halt;
            # already-cleared vehicles are never touched.
            for vehicle in vehicles:
                if vehicle.name in cleared_vehicles:
                    continue  # already exited — don't interfere
                d = dist_to_intersection[vehicle.name]
                if vehicle.name == priority_vehicle_name:
                    vehicle.target_speed_mps = vehicle.base_speed_mps
                else:
                    if d > INTERSECTION_STOP_M:
                        # Linear taper between WARNING and STOP radii
                        ratio = (d - INTERSECTION_STOP_M) / (
                            INTERSECTION_WARNING_M - INTERSECTION_STOP_M
                        )
                        vehicle.target_speed_mps = vehicle.base_speed_mps * max(0.05, ratio)
                    else:
                        vehicle.target_speed_mps = 0.0

            # Check if priority vehicle has cleared the intersection
            if priority_vehicle:
                d = dist_to_intersection[priority_vehicle_name]
                # Track when priority vehicle actually enters the intersection box
                if d < INTERSECTION_STOP_M:
                    priority_in_intersection = True
                # Only start clearing timer once it has entered AND exited
                if priority_in_intersection and d > INTERSECTION_CLEAR_M:
                    if clear_confirm_start == 0.0:
                        clear_confirm_start = now
                    elif now - clear_confirm_start > 0.2:
                        print(f"\n✅ {priority_vehicle_name} cleared — releasing others")
                        state = "CLEARING"
                        clear_confirm_start = 0.0
                else:
                    clear_confirm_start = 0.0

        # ── CLEARING ─────────────────────────────────────────────────────────
        elif state == "CLEARING":
            rsu.publish_denm(
                cause_code=DENM_CAUSE_COLLISION_RISK,
                sub_cause_code=0,
                validity_duration=5,
                notify_vehicles=vehicles,
            )
            cleared_vehicles.add(priority_vehicle_name)
            for vehicle in vehicles:
                vehicle.target_speed_mps = vehicle.base_speed_mps
                vehicle.in_collision_avoidance = False
            priority_vehicle_name = ""
            state = "NORMAL"
            print("🟢 Intersection clear — all vehicles resuming\n")

        # ── Periodic status print (once per second) ───────────────────────────
        tick_count += 1
        if tick_count % int(TICK_HZ) == 0:
            print(f"  [{state}] priority={priority_vehicle_name or '—'}")
            for v in vehicles:
                d = dist_to_intersection[v.name]
                print(
                    f"  {v.name}  lat={v.current_lat:.6f}  lon={v.current_lon:.6f}"
                    f"  spd={v.current_speed_mps:.1f}m/s  d_int={d:.1f}m"
                )

        elapsed = time.time() - tick_start
        time.sleep(max(0.0, TICK_SECONDS - elapsed))

    for vehicle in vehicles:
        vehicle.close()
    rsu.close()
    print("Simulator stopped")
