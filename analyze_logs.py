#!/usr/bin/env python3
"""Analyze logged CAM messages and compare with expected routes."""

import json
import sys
from pathlib import Path
import math

# Define expected routes (from simulador.py)
EXPECTED_ROUTES = {
    "obu1": [
        (40.62835, -8.65439),
        (40.62860, -8.65439),  # frente
        (40.62860, -8.65410),  # direita
        (40.62885, -8.65410),  # frente
        (40.62885, -8.65385),  # direita
        (40.62905, -8.65385),  # frente
        (40.62905, -8.65360),  # direita
        (40.62880, -8.65360),  # trás
        (40.62880, -8.65335),  # direita
        (40.62855, -8.65335),  # trás
        (40.62855, -8.65360),  # esquerda
    ],
    "obu2": [
        (40.62810, -8.65485),
        (40.62830, -8.65485),  # frente
        (40.62850, -8.65485),  # frente
        (40.62850, -8.65460),  # direita
        (40.62850, -8.65435),  # direita
        (40.62870, -8.65435),  # frente
        (40.62890, -8.65435),  # frente
        (40.62890, -8.65455),  # esquerda
        (40.62890, -8.65475),  # esquerda
        (40.62910, -8.65475),  # frente
        (40.62920, -8.65475),  # frente
        (40.62920, -8.65410),  # direita
        (40.62920, -8.65345),  # direita
    ],
}

def haversine_distance(lat1, lon1, lat2, lon2):
    """Distance in meters between two WGS84 coordinates."""
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

def find_closest_waypoint(lat, lon, route):
    """Find closest waypoint in route and distance in meters."""
    min_dist = float('inf')
    closest_idx = 0
    
    for idx, (rlat, rlon) in enumerate(route):
        dist = haversine_distance(lat, lon, rlat, rlon)
        if dist < min_dist:
            min_dist = dist
            closest_idx = idx
    
    return closest_idx, min_dist

def analyze_obu(obu_name):
    """Analyze CAM logs for an OBU."""
    log_file = f"logs/{obu_name}_cam_log.jsonl"
    
    if not Path(log_file).exists():
        print(f"❌ Log file not found: {log_file}")
        return
    
    print(f"\n{'='*60}")
    print(f" Analysis for {obu_name.upper()}")
    print(f"{'='*60}")
    
    entries = []
    with open(log_file, "r") as f:
        for line in f:
            try:
                entries.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    
    if not entries:
        print(f" No valid log entries found")
        return
    
    route = EXPECTED_ROUTES.get(obu_name)
    if not route:
        print(f" No expected route defined for {obu_name}")
        return
    
    print(f"\n Total CAM messages: {len(entries)}")
    print(f" Route waypoints: {len(route)}")
    print(f" Time span: {entries[0]['timestamp']} → {entries[-1]['timestamp']}")
    
    # Analyze route coverage
    print(f"\n--- Route Coverage Analysis ---")
    distances_to_route = []
    
    for i, entry in enumerate(entries):
        lat = entry["lat"]
        lon = entry["lon"]
        
        closest_idx, dist = find_closest_waypoint(lat, lon, route)
        distances_to_route.append(dist)
        
        # Print every 10th message for brevity
        if i % 10 == 0 or i < 3:
            waypoint = route[closest_idx]
            print(
                f"  [{i:3d}] lat={lat:.6f} lon={lon:.6f} "
                f"→ Closest waypoint #{closest_idx} at {dist:.1f}m"
            )
    
    # Statistics
    avg_dist = sum(distances_to_route) / len(distances_to_route)
    max_dist = max(distances_to_route)
    
    print(f"\n--- Route Accuracy ---")
    print(f"  ✓ Average distance to route: {avg_dist:.2f} m")
    print(f"  ✓ Maximum deviation: {max_dist:.2f} m")
    
    if avg_dist < 5.0:
        print(f"  ✅ Excellent: Position matches route closely")
    elif avg_dist < 15.0:
        print(f"  ✅ Good: Position follows route")
    else:
        print(f"  ⚠️  Deviation from expected route")
    
    # Check speed and heading consistency
    speeds = [e["speed"] for e in entries if e["speed"] is not None]
    headings = [e["heading"] for e in entries if e["heading"] is not None]
    
    if speeds:
        avg_speed = sum(speeds) / len(speeds)
        print(f"\n--- Vehicle Dynamics ---")
        print(f"  Speed: {min(speeds):.1f} - {max(speeds):.1f} m/s (avg: {avg_speed:.1f} m/s)")
    
    if headings:
        print(f"  Heading: {min(headings):.0f}° - {max(headings):.0f}°")

if __name__ == "__main__":
    analyze_obu("obu1")
    analyze_obu("obu2")
    
    print(f"\n{'='*60}")
    print("📁 Full logs saved in: logs/")
    print("   - logs/obu1_cam_log.jsonl")
    print("   - logs/obu2_cam_log.jsonl")
    print(f"{'='*60}\n")
