#!/usr/bin/env python3
"""Analyze CAM logs using local step-by-step motion prediction.

This checks whether each CAM is consistent with the previous one using speed,
heading and the observed time delta. It does not validate route turns exactly,
because the current logs do not store route/segment state.
"""

from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path


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


def destination_point(lat, lon, heading_deg, distance_m):
    """Project a WGS84 point forward along a bearing for a given distance."""
    r = 6371000.0
    angular_distance = distance_m / r
    bearing = math.radians(heading_deg)
    phi1 = math.radians(lat)
    lambda1 = math.radians(lon)

    phi2 = math.asin(
        math.sin(phi1) * math.cos(angular_distance)
        + math.cos(phi1) * math.sin(angular_distance) * math.cos(bearing)
    )
    lambda2 = lambda1 + math.atan2(
        math.sin(bearing) * math.sin(angular_distance) * math.cos(phi1),
        math.cos(angular_distance) - math.sin(phi1) * math.sin(phi2),
    )

    return math.degrees(phi2), (math.degrees(lambda2) + 540.0) % 360.0 - 180.0


def parse_timestamp(value):
    """Parse the ISO timestamp written by the logger."""
    if not value:
        return None
    return datetime.fromisoformat(str(value).replace("Z", "+00:00"))


def analyze_obu(obu_name):
    """Analyze CAM logs for an OBU using local kinematic prediction."""
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
        print(" No valid log entries found")
        return

    print(f"\n Total CAM messages: {len(entries)}")
    print(f" Time span: {entries[0]['timestamp']} → {entries[-1]['timestamp']}")

    paired_steps = []

    for current, following in zip(entries, entries[1:]):
        current_ts = parse_timestamp(current.get("timestamp"))
        following_ts = parse_timestamp(following.get("timestamp"))

        if current_ts is None or following_ts is None:
            continue

        dt = (following_ts - current_ts).total_seconds()
        if dt <= 0:
            continue

        speed = current.get("speed")
        heading = current.get("heading")
        if speed is None or heading is None:
            continue

        predicted_lat, predicted_lon = destination_point(
            current["lat"],
            current["lon"],
            heading,
            speed * dt,
        )

        actual_step_m = haversine_distance(
            current["lat"],
            current["lon"],
            following["lat"],
            following["lon"],
        )
        predicted_error_m = haversine_distance(
            predicted_lat,
            predicted_lon,
            following["lat"],
            following["lon"],
        )
        observed_speed = actual_step_m / dt if dt > 0 else 0.0

        paired_steps.append(
            {
                "dt": dt,
                "speed": speed,
                "heading": heading,
                "actual_step_m": actual_step_m,
                "predicted_error_m": predicted_error_m,
                "observed_speed": observed_speed,
            }
        )

    if not paired_steps:
        print(" No usable consecutive CAM pairs found")
        return

    regular_steps = [step for step in paired_steps if 0.05 <= step["dt"] <= 0.5]
    skipped_steps = len(paired_steps) - len(regular_steps)
    if not regular_steps:
        regular_steps = paired_steps
        skipped_steps = 0

    print("\n--- Step Prediction Analysis ---")
    for index, step in enumerate(regular_steps):
        if index < 3 or index % 10 == 0:
            print(
                f"  [{index:3d}] dt={step['dt']:.3f}s "
                f"speed={step['speed']:.1f}m/s heading={step['heading']:.0f}° "
                f"step={step['actual_step_m']:.2f}m "
                f"prediction_error={step['predicted_error_m']:.2f}m"
            )

    avg_dt = sum(step["dt"] for step in regular_steps) / len(regular_steps)
    avg_speed = sum(step["speed"] for step in regular_steps) / len(regular_steps)
    avg_step = sum(step["actual_step_m"] for step in regular_steps) / len(regular_steps)
    avg_error = sum(step["predicted_error_m"] for step in regular_steps) / len(regular_steps)
    max_error = max(step["predicted_error_m"] for step in regular_steps)
    avg_observed_speed = sum(step["observed_speed"] for step in regular_steps) / len(regular_steps)

    print("\n--- Kinematic Consistency ---")
    print(f"  Regular steps analyzed: {len(regular_steps)} / {len(paired_steps)}")
    if skipped_steps:
        print(f"  Skipped gaps/outliers: {skipped_steps}")
    print(f"  Tick interval: {avg_dt:.3f} s")
    print(f"  Logged speed: {avg_speed:.2f} m/s")
    print(f"  Observed step speed: {avg_observed_speed:.2f} m/s")
    print(f"  Mean step length: {avg_step:.2f} m")
    print(f"  Mean next-CAM prediction error: {avg_error:.3f} m")
    print(f"  Max next-CAM prediction error: {max_error:.3f} m")

    if avg_error < 0.5:
        print("  ✅ Excelente: a trajetória está consistente com speed + heading")
    elif avg_error < 2.0:
        print("  ✅ Bom: a previsão bate razoavelmente com o próximo CAM")
    else:
        print("  ⚠️  Erro alto: vale a pena rever a cinemática ou os timestamps")

    print(
        "  Nota: este log não guarda estado de rota/segmento, por isso não dá "
        "para validar viragens de forma exacta só com estes campos."
    )


if __name__ == "__main__":
    analyze_obu("obu1")
    analyze_obu("obu2")

    print(f"\n{'='*60}")
    print(" Full logs saved in: logs/")
    print("   - logs/obu1_cam_log.jsonl")
    print("   - logs/obu2_cam_log.jsonl")
    print(f"{'='*60}\n")
