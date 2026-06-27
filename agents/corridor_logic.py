"""Module for emergency corridor operations and dynamic lane-change maneuvers.

Functions receive the 'agent' instance as their first argument to manipulate
route definitions, offsets, and speed constraints in the central CarAgent.
"""
from __future__ import annotations

import math
import time
from types import SimpleNamespace
from typing import Optional

import simulator_core as core
from simulator_core import (
    haversine_meters,
    bearing_degrees,
    heading_delta_degrees,
    interpolate,
    DENM_CAUSE_EMERGENCY,
)

# Raio da Terra (WGS-84) para conversão local de metros para graus decimais
_EARTH_RADIUS_M = 6378137.0


def offset_route(route: list[tuple[float, float]], lane_type: str, offset_m: float) -> list[tuple[float, float]]:
    """Returns a copy of the route laterally shifted by a given offset in meters

    towards the specified lane direction ('left' or 'right').
    """
    if lane_type not in ("left", "right") or len(route) < 2:
        return list(route)

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


def advance_vehicle_by_meters(agent, distance_m: float) -> None:
    """Teleports or advances the vehicle position along its current active route

    by a fixed distance in meters, syncing internal routing indices.
    """
    remaining = distance_m
    for i in range(len(agent.vehicle.route) - 1):
        p1 = agent.vehicle.route[i]
        p2 = agent.vehicle.route[i + 1]
        seg_dist = haversine_meters(*p1, *p2)
        if remaining <= seg_dist:
            ratio = remaining / max(seg_dist, 0.01)
            agent.vehicle.current_lat, agent.vehicle.current_lon = interpolate(*p1, *p2, ratio)
            agent.vehicle.segment_idx = i
            agent.vehicle.last_heading_deg = bearing_degrees(*p1, *p2)
            agent.base_segment_idx = i
            return
        remaining -= seg_dist

    agent.vehicle.current_lat, agent.vehicle.current_lon = agent.vehicle.route[-1]
    agent.vehicle.segment_idx = len(agent.vehicle.route) - 1
    agent.base_segment_idx = max(len(agent.base_route) - 1, 0)


def change_to_right_lane(agent) -> None:
    """Splices in a smooth lateral merge path from the vehicle's current position

    to the right-shifted base route representation.
    """
    if agent.current_lane in ("right", "changing"):
        return

    agent.current_lane = "changing"

    cur_lat, cur_lon = agent.vehicle.current_lat, agent.vehicle.current_lon
    merge_distance_m = 20.0

    # Projeta a posição lateral atual sobre o segmento da rota base original
    seg_start = agent.base_route[agent.base_segment_idx]
    next_base_idx = min(agent.base_segment_idx + 1, len(agent.base_route) - 1)
    seg_end = agent.base_route[next_base_idx]
    seg_len = max(haversine_meters(*seg_start, *seg_end), 0.01)

    seg_heading = bearing_degrees(*seg_start, *seg_end)
    bearing_to_cur = bearing_degrees(*seg_start, cur_lat, cur_lon)
    raw_distance = haversine_meters(*seg_start, cur_lat, cur_lon)
    along_distance = raw_distance * math.cos(
        math.radians(heading_delta_degrees(seg_heading, bearing_to_cur))
    )
    along_distance = min(max(along_distance, 0.0), seg_len)

    # Avança na rota base para encontrar o ponto de ancoragem do merge
    target_dist = along_distance + merge_distance_m
    accumulated = 0.0
    merge_point: Optional[tuple] = None
    next_idx_after_merge = len(agent.base_route)

    for i in range(agent.base_segment_idx, len(agent.base_route) - 1):
        p1 = agent.base_route[i]
        p2 = agent.base_route[i + 1]
        s_dist = max(haversine_meters(*p1, *p2), 0.01)
        if accumulated + s_dist >= target_dist:
            ratio = (target_dist - accumulated) / s_dist
            mp_lat, mp_lon = interpolate(*p1, *p2, ratio)
            
            # Desvia perpendicularmente para a direita com base no azimute do segmento
            h = bearing_degrees(*p1, *p2)
            offset_h = (h + 90) % 360
            d_lat = math.cos(math.radians(offset_h)) * agent.lane_offset_m / _EARTH_RADIUS_M
            d_lon = (
                math.sin(math.radians(offset_h)) * agent.lane_offset_m
                / (_EARTH_RADIUS_M * math.cos(math.radians(mp_lat)))
            )
            merge_point = (mp_lat + math.degrees(d_lat), mp_lon + math.degrees(d_lon))
            next_idx_after_merge = i + 1
            break
        accumulated += s_dist

    if merge_point is None:
        remaining_base = agent.base_route[agent.base_segment_idx + 1:]
        if not remaining_base:
            return
        shifted_right = offset_route(remaining_base, "right", agent.lane_offset_m)
        new_route = [(cur_lat, cur_lon)] + shifted_right
    else:
        remaining_base = agent.base_route[next_idx_after_merge:]
        shifted_right = offset_route(remaining_base, "right", agent.lane_offset_m)
        new_route = [(cur_lat, cur_lon), merge_point] + shifted_right

    if len(new_route) < 2:
        return

    agent.vehicle.set_route(new_route, keep_current_position=True)
    # Garante momento dinâmico durante a transição de faixa (70% da velocidade)
    agent.vehicle.target_speed_mps = agent.vehicle.base_speed_mps * 0.7


def publish_emergency_denm(agent) -> None:
    """Generates and broadcasts a periodic emergency vehicle warning DENM (cause 95)

    to both local and peer infrastructure/vehicle components.
    """
    notify = [SimpleNamespace(client=agent.vehicle.client)]
    if agent.peer_clients:
        notify.extend(SimpleNamespace(client=c) for c in agent.peer_clients)

    core.publish_denm(
        agent.vehicle,
        cause_code=DENM_CAUSE_EMERGENCY,
        sub_cause_code=agent.corridor.emergency_denm_sub_cause,
        validity_duration=agent.corridor.emergency_denm_validity_s,
        event_position=(agent.vehicle.current_lat, agent.vehicle.current_lon),
        notify_vehicles=notify,
    )


def react_to_emergency_vehicle(agent, event_position: tuple) -> None:
    ev_lat = event_position[0]
    ev_lon = event_position[1]
    if ev_lat is None or ev_lon is None:
        return

    dist = haversine_meters(agent.vehicle.current_lat, agent.vehicle.current_lon, ev_lat, ev_lon)
    bearing_to_ev = bearing_degrees(agent.vehicle.current_lat, agent.vehicle.current_lon, ev_lat, ev_lon)
    heading_diff = heading_delta_degrees(agent.vehicle.last_heading_deg, bearing_to_ev)
    is_behind = abs(heading_diff) > 90.0

    # Reage apenas se o veículo de emergência estiver de facto a aproximar-se por trás
    if is_behind and dist <= agent.corridor.yield_distance_threshold_m:
        agent.yield_until = time.time() + agent.corridor.yield_duration_s
        
        if agent.current_lane == "left":
            change_to_right_lane(agent)
            print(f"[{agent.name}] Veículo de Emergência detetado atrás ({dist:.1f}m) -> Movendo para a faixa da direita")
        elif agent.current_lane == "right":
            agent.vehicle.target_speed_mps = agent.vehicle.base_speed_mps * 0.4
            print(f"[{agent.name}] Veículo de Emergência detetado atrás ({dist:.1f}m) -> Abrandando na faixa da direita")
