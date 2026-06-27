"""Module for parsing and decoding CAM and DENM messages.

Pure functions only, maintaining no internal state.
"""
from __future__ import annotations

from types import SimpleNamespace
from typing import Optional


def extract_position_from_cam(cam_payload: dict) -> Optional[tuple[float, float]]:
    """Extracts (latitude, longitude) from a standard CAM payload."""
    try:
        fields = cam_payload.get("fields", {})
        cam = fields.get("cam", {})
        cam_pva = cam.get("camParameters", {}).get("basicContainer", {}).get("referencePosition", {})
        
        lat = cam_pva.get("latitude")
        lon = cam_pva.get("longitude")
        
        if lat is not None and lon is not None:
            return float(lat), float(lon)
    except (AttributeError, TypeError, ValueError):
        pass
    return None


def extract_heading_from_cam(cam_payload: dict) -> float:
    """Extracts heading value from a standard CAM payload. Default is 0.0."""
    try:
        fields = cam_payload.get("fields", {})
        cam = fields.get("cam", {})
        high_freq = cam.get("camParameters", {}).get("highFrequencyContainer", {})
        heading_container = high_freq.get("basicVehicleContainerHighFrequency", {}).get("heading", {})
        
        heading = heading_container.get("headingValue")
        if heading is not None:
            return float(heading)
    except (AttributeError, TypeError, ValueError):
        pass
    return 0.0


def build_peer_vehicle_view(cam_payload: dict) -> Optional[SimpleNamespace]:
    """Parses a raw CAM message from another vehicle and builds a normalized 
    SimpleNamespace view representing its current real-time state.
    """
    station_id = cam_payload.get("fields", {}).get("header", {}).get("stationId")
    if station_id is None:
        return None

    pos = extract_position_from_cam(cam_payload)
    if not pos:
        return None

    lat, lon = pos
    heading = extract_heading_from_cam(cam_payload)

    # Extração opcional de velocidade se disponível no highFrequencyContainer
    speed_kmh = 0.0
    try:
        fields = cam_payload.get("fields", {})
        cam = fields.get("cam", {})
        high_freq = cam.get("camParameters", {}).get("highFrequencyContainer", {})
        speed_val = high_freq.get("basicVehicleContainerHighFrequency", {}).get("speed", {}).get("speedValue", 0)
        if speed_val is not None:
            # ETSI usa 0.01 m/s como unidade
            speed_ms = float(speed_val) * 0.01  # Converter para m/s
            speed_kmh = speed_ms * 3.6  # Converter m/s para km/h
    except (AttributeError, TypeError, ValueError):
        pass

    # Cria uma estrutura leve e agnóstica para o CarAgent manipular internamente
    return SimpleNamespace(
        station_id=int(station_id),
        latitude=lat,
        longitude=lon,
        heading=heading,
        speed_kmh=speed_kmh
    )
