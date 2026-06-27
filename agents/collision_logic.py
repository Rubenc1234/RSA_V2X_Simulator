"""Module for collision detection, mitigation, and DENM reaction behaviors.
Pure geometric approach - uses heading, bearing, and distance only.

DEBUG VERSION - logs everything to help diagnose issues.
"""
from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Optional

import simulator_core as core
from simulator_core import (
    WARNING_DISTANCE_M,
    YIELD_DISTANCE_M,
    CLEAR_DISTANCE_M,
    DENM_CAUSE_COLLISION_RISK,
    choose_yield_vehicle,
    haversine_meters,
    bearing_degrees,
)


def _get_compatible_peer_view(agent) -> Optional[SimpleNamespace]:
    """Get nearest peer from MQTT-received CAM data."""
    with agent.peer_lock:
        if not agent.peer_vehicles:
            return None

        my_lat = agent.vehicle.current_lat
        my_lon = agent.vehicle.current_lon

        closest_peer = None
        min_dist = float('inf')

        for station_id, peer in agent.peer_vehicles.items():
            dist = haversine_meters(my_lat, my_lon, peer.latitude, peer.longitude)
            if dist < min_dist:
                min_dist = dist
                closest_peer = peer

        if not closest_peer:
            return None

        lat, lon, heading = closest_peer.latitude, closest_peer.longitude, closest_peer.heading

        # Estima um pequeno troço de rota (300m antes/depois) a partir da
        # posição e heading instantâneos, só para choose_yield_vehicle ter
        # "distância à origem/destino" com que comparar.
        def _destination_point(lat0, lon0, bearing_deg, distance_m=300):
            import math
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

        est_start = _destination_point(lat, lon, (heading + 180) % 360)
        est_end = _destination_point(lat, lon, heading)

        return SimpleNamespace(
            station_id=closest_peer.station_id,
            name=f"obu_{closest_peer.station_id}",
            current_lat=lat,
            current_lon=lon,
            last_heading_deg=heading,
            current_speed_mps=closest_peer.speed_kmh / 3.6,
            distance_from_route_start_m=lambda: haversine_meters(*est_start, lat, lon),
            distance_to_route_end_m=lambda: haversine_meters(lat, lon, *est_end),
        )


def _normalize_angle_diff(angle1: float, angle2: float) -> float:
    """Calculate shortest angle difference between two bearings (0-180)."""
    diff = abs(angle1 - angle2)
    if diff > 180:
        diff = 360 - diff
    return diff


def _is_heading_opposite(my_heading: float, peer_heading: float, tolerance_deg: float = 90) -> bool:
    """Check if two vehicles are heading in roughly opposite directions."""
    diff = _normalize_angle_diff(my_heading, peer_heading)
    result = diff >= (180 - tolerance_deg) and diff <= (180 + tolerance_deg)
    return result


def _is_bearing_towards_peer(my_heading: float, bearing_to_peer: float, tolerance_deg: float = 60) -> bool:
    """Check if I'm heading towards the peer (bearing alignment)."""
    diff = _normalize_angle_diff(my_heading, bearing_to_peer)
    result = diff <= tolerance_deg
    return result


def _is_distance_decreasing(
    agent,
    peer_vehicle,
    distance_now: float,
    history_window_s: float = 1.0
) -> bool:
    """Estimate if distance is decreasing based on current speeds and bearings."""
    
    bearing_to_peer = bearing_degrees(
        agent.vehicle.current_lat,
        agent.vehicle.current_lon,
        peer_vehicle.current_lat,
        peer_vehicle.current_lon
    )
    
    # Relative velocity approach along bearing
    my_speed_towards_peer = agent.vehicle.current_speed_mps * (
        1.0 if _is_bearing_towards_peer(agent.vehicle.last_heading_deg, bearing_to_peer, tolerance_deg=120)
        else 0.0
    )
    
    # Estimate peer speed towards us
    bearing_to_us = bearing_degrees(
        peer_vehicle.current_lat,
        peer_vehicle.current_lon,
        agent.vehicle.current_lat,
        agent.vehicle.current_lon
    )
    peer_speed_towards_us = peer_vehicle.current_speed_mps * (
        1.0 if _is_bearing_towards_peer(peer_vehicle.last_heading_deg, bearing_to_us, tolerance_deg=120)
        else 0.0
    )
    
    # Combined approach speed
    closing_speed = my_speed_towards_peer + peer_speed_towards_us
    
    # Distance decreases if closing_speed > 0
    return closing_speed > 0.1


def run_detection_tick(agent) -> None:
    """Geometric collision detection loop using heading + bearing + distance."""
    
    if not agent.collision.enabled:
        return

    try:
        # ════════════════════════════════════════════════════════
        # STEP 1: GET PEER DATA
        # ════════════════════════════════════════════════════════
        
        peer_vehicle = _get_compatible_peer_view(agent)
        
        with agent.peer_lock:
            peer_count = len(agent.peer_vehicles)
        
        if not peer_vehicle:
            # Silencioso se não há peers
            return
        
        # ════════════════════════════════════════════════════════
        # STEP 2: CALCULATE GEOMETRY
        # ════════════════════════════════════════════════════════
        
        my_lat = agent.vehicle.current_lat
        my_lon = agent.vehicle.current_lon
        peer_lat = peer_vehicle.current_lat
        peer_lon = peer_vehicle.current_lon
        
        distance = haversine_meters(my_lat, my_lon, peer_lat, peer_lon)
        bearing_to_peer = bearing_degrees(my_lat, my_lon, peer_lat, peer_lon)
        
        my_heading = agent.vehicle.last_heading_deg
        my_speed = agent.vehicle.current_speed_mps
        peer_heading = peer_vehicle.last_heading_deg
        peer_speed = peer_vehicle.current_speed_mps
        
        # ════════════════════════════════════════════════════════
        # STEP 3: CHECK COLLISION CONDITIONS
        # ════════════════════════════════════════════════════════
        
        heading_opposite = _is_heading_opposite(my_heading, peer_heading, tolerance_deg=90)
        bearing_aligned = _is_bearing_towards_peer(my_heading, bearing_to_peer, tolerance_deg=60)
        distance_decreasing = _is_distance_decreasing(agent, peer_vehicle, distance)
        
        in_collision_risk = (
            heading_opposite and 
            bearing_aligned and 
            distance_decreasing and
            distance < 300
        )
        
        # ════════════════════════════════════════════════════════
        # STEP 4: LOG EVERYTHING (for diagnosis)
        # ════════════════════════════════════════════════════════
        
        if distance < 500:  # Log nearby vehicles
            heading_diff = _normalize_angle_diff(my_heading, peer_heading)
            bearing_diff = _normalize_angle_diff(my_heading, bearing_to_peer)
            
            print(f"\n[{agent.name}] ═════════════════════════════════════════════")
            print(f"[{agent.name}] COLLISION DETECTION TICK")
            print(f"[{agent.name}] ─────────────────────────────────────────────")
            print(f"[{agent.name}] Peer: {peer_vehicle.name}")
            print(f"[{agent.name}] Distance: {distance:.1f}m")
            print(f"[{agent.name}] ─────────────────────────────────────────────")
            print(f"[{agent.name}] MY HEADING:      {my_heading:.1f}°")
            print(f"[{agent.name}] PEER HEADING:    {peer_heading:.1f}°")
            print(f"[{agent.name}] HEADING DIFF:    {heading_diff:.1f}° (opposite if ~180°)")
            print(f"[{agent.name}] └─> is_opposite: {heading_opposite} (tolerance ±90°)")
            print(f"[{agent.name}] ─────────────────────────────────────────────")
            print(f"[{agent.name}] MY BEARING TO PEER: {bearing_to_peer:.1f}°")
            print(f"[{agent.name}] BEARING DIFF:       {bearing_diff:.1f}° (aligned if ~0°)")
            print(f"[{agent.name}] └─> is_aligned:     {bearing_aligned} (tolerance ±60°)")
            print(f"[{agent.name}] ─────────────────────────────────────────────")
            print(f"[{agent.name}] MY SPEED:        {my_speed:.1f} m/s")
            print(f"[{agent.name}] PEER SPEED:      {peer_speed:.1f} m/s")
            print(f"[{agent.name}] └─> decreasing:  {distance_decreasing}")
            print(f"[{agent.name}] ─────────────────────────────────────────────")
            print(f"[{agent.name}] IN_COLLISION_RISK: {in_collision_risk}")
            print(f"[{agent.name}] WARNING_DISTANCE:  {WARNING_DISTANCE_M}m")
            print(f"[{agent.name}] ═════════════════════════════════════════════\n")
        
        # ════════════════════════════════════════════════════════
        # STEP 5: SAFE MODE ACTIVATION
        # ════════════════════════════════════════════════════════
        
        if in_collision_risk and distance <= WARNING_DISTANCE_M:
            print(f"\n>>> [{agent.name}] 🚨 COLLISION RISK DETECTED! distance={distance:.1f}m <<<\n")
            
            # ✅ Sem choose_yield_vehicle() - abrandamos todos
            agent.yield_vehicle_name = peer_vehicle.name if hasattr(peer_vehicle, 'name') else f"peer_{peer_vehicle.station_id}"
            agent.avoidance_active = True
            agent.yield_mode = "collision_risk"
            agent.denm_hold_until = time.time() + 3.0
            
            # Speed reduction - abrandar bastante para evitar colisão
            agent.vehicle.target_speed_mps = max(1.0, agent.vehicle.base_speed_mps * 0.3)
            print(f"[{agent.name}] ⚠️ SAFE MODE: slowing to {agent.vehicle.target_speed_mps:.1f}m/s")
            
            # Publish collision risk DENM
            publish_ok = (time.time() - agent.last_published_time) > 1.0
            if publish_ok and agent.collision.emit_collision_risk_denm:
                print(f"[{agent.name}] 📢 Publishing collision risk DENM...")
                core.publish_denm(
                    agent.vehicle,
                    cause_code=DENM_CAUSE_COLLISION_RISK,
                    sub_cause_code=0,
                    validity_duration=2,
                    event_position=(my_lat, my_lon),
                )
                agent.last_published_time = time.time()
                agent.last_published_event_pos = (my_lat, my_lon)
                print(f"[{agent.name}] ✅ DENM published!")
            else:
                print(f"[{agent.name}] ⏭️ DENM publish skipped (throttle or disabled)")
        
        # ════════════════════════════════════════════════════════
        # STEP 6: SAFE MODE CLEARANCE
        # ════════════════════════════════════════════════════════
        
        elif agent.avoidance_active and agent.yield_mode == "collision_risk":
            if not in_collision_risk or distance > CLEAR_DISTANCE_M:
                agent.avoidance_active = False
                agent.yield_vehicle_name = ""
                agent.yield_mode = "stop"
                agent.vehicle.target_speed_mps = agent.vehicle.base_speed_mps
                print(f"[{agent.name}] ✓ Collision risk cleared, resuming normal speed")
    
    except Exception as e:
        print(f"\n[{agent.name}] ❌ ERROR in collision detection: {e}\n")
        import traceback
        traceback.print_exc()


def apply_collision_risk_reaction(agent, peer_vehicle, event_lat: float, event_lon: float) -> None:
    """Handle incoming DENM collision risk warnings."""
    
    print(f"[{agent.name}] apply_collision_risk_reaction called")
    
    if not peer_vehicle:
        print(f"[{agent.name}]   peer_vehicle is None, returning")
        return

    # ✅ Sem choose_yield_vehicle() - apenas abrandamos bastante
    agent.yield_vehicle_name = peer_vehicle.name if hasattr(peer_vehicle, 'name') else f"peer_{peer_vehicle.station_id}"
    agent.avoidance_active = True
    agent.vehicle.target_speed_mps = agent.vehicle.base_speed_mps * 0.5  # Abrandar a 50%

    event_distance = haversine_meters(
        agent.vehicle.current_lat, agent.vehicle.current_lon, 
        event_lat, event_lon
    )

    print(f"[{agent.name}]   event_distance={event_distance:.1f}m, ignore_distance={agent.collision.denm_ignore_distance_m}m")

    if event_distance > agent.collision.denm_ignore_distance_m:
        print(f"[{agent.name}]   distance too far, ignoring")
        return

    agent.yield_mode = "denm_risk"
    agent.denm_hold_until = time.time() + 4.0

    if event_distance > agent.collision.denm_slowdown_distance_m:
        factor = 0.85 if agent.vehicle.name == agent.yield_vehicle_name else 0.95
        agent.vehicle.target_speed_mps = agent.vehicle.base_speed_mps * factor
        print(f"[{agent.name}]   DENM risk (far): slowing to {agent.vehicle.target_speed_mps:.1f}m/s")
    
    elif event_distance > agent.collision.denm_stop_distance_m:
        factor = 0.4 if agent.vehicle.name == agent.yield_vehicle_name else 0.7
        agent.vehicle.target_speed_mps = agent.vehicle.base_speed_mps * factor
        print(f"[{agent.name}]   DENM risk (medium): slowing to {agent.vehicle.target_speed_mps:.1f}m/s")
    
    else:
        agent.vehicle.target_speed_mps = max(0.5, agent.vehicle.base_speed_mps * 0.2)
        print(f"[{agent.name}]   DENM risk (CLOSE): emergency slowdown to {agent.vehicle.target_speed_mps:.1f}m/s")


def apply_accident_reaction(agent, peer_vehicle, event_lat: float, event_lon: float) -> None:
    """Handle incoming DENM accident warnings."""
    
    print(f"[{agent.name}] apply_accident_reaction called")
    
    event_distance = haversine_meters(
        agent.vehicle.current_lat, agent.vehicle.current_lon, 
        event_lat, event_lon
    )

    if event_distance > agent.accident.accident_ignore_distance_m:
        print(f"[{agent.name}]   distance too far, ignoring")
        return

    agent.yield_until = time.time() + agent.accident.incident_reset_seconds

    if event_distance <= agent.accident.accident_stop_distance_m:
        if agent.accident.hard_stop_on_accident:
            agent.vehicle.target_speed_mps = 0.0
            agent.vehicle.current_speed_mps = 0.0
            agent.avoidance_active = True
            agent.yield_mode = "accident"
            agent.denm_hold_until = time.time() + 5.0
            print(f"[{agent.name}] 🚨 ACCIDENT detected {event_distance:.1f}m -> HARD STOP")
        
        elif agent.accident.reroute_on_accident:
            end_lat, end_lon = agent.vehicle.end_point
            agent.vehicle.replan_route_to(
                end_lat, end_lon, 
                use_alternative=agent.accident.reroute_prefer_alternative
            )
            print(f"[{agent.name}] 🚨 ACCIDENT {event_distance:.1f}m -> replanning route")
    
    else:
        agent.vehicle.target_speed_mps = agent.vehicle.base_speed_mps * 0.5
        print(f"[{agent.name}] Approaching accident {event_distance:.1f}m -> slowdown (50%)")


def apply_soft_yield_reaction(
    agent, 
    peer_vehicle, 
    event_lat: Optional[float], 
    event_lon: Optional[float]
) -> None:
    """Fallback for unmapped DENM types."""
    
    print(f"[{agent.name}] apply_soft_yield_reaction called")
    
    if not peer_vehicle:
        print(f"[{agent.name}]   peer_vehicle is None, returning")
        return

    # ✅ Sem choose_yield_vehicle() - apenas abrandamos
    agent.yield_vehicle_name = peer_vehicle.name if hasattr(peer_vehicle, 'name') else f"peer_{peer_vehicle.station_id}"
    agent.avoidance_active = True
    agent.vehicle.target_speed_mps = agent.vehicle.base_speed_mps * 0.7  # Abrandar a 70%
    
    if event_lat is not None and event_lon is not None:
        event_distance = haversine_meters(
            agent.vehicle.current_lat, agent.vehicle.current_lon, 
            event_lat, event_lon
        )
    else:
        event_distance = haversine_meters(
            agent.vehicle.current_lat, agent.vehicle.current_lon, 
            peer_vehicle.current_lat, peer_vehicle.current_lon
        )

    agent.yield_mode = "soft_yield"
    agent.denm_hold_until = time.time() + 6.0

    slow_factor_yield = 0.35
    slow_factor_other = 0.6

    if agent.vehicle.name == agent.yield_vehicle_name:
        agent.vehicle.target_speed_mps = max(0.5, agent.vehicle.base_speed_mps * slow_factor_yield)
    else:
        if event_distance <= YIELD_DISTANCE_M:
            agent.vehicle.target_speed_mps = max(
                0.6, 
                agent.vehicle.base_speed_mps * (slow_factor_other - 0.15)
            )
        else:
            agent.vehicle.target_speed_mps = max(1.0, agent.vehicle.base_speed_mps * slow_factor_other)

    print(f"[{agent.name}] Soft yield -> {agent.yield_vehicle_name} ({agent.yield_mode}); dist={event_distance:.1f}m")
