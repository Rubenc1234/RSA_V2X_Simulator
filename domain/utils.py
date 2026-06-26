import math
import time
from typing import Dict, Any, Tuple

# ──────────────────────────────────────────────────────────────────────
# 1. FUNÇÕES GEOGRÁFICAS / MATEMÁTICAS
# ──────────────────────────────────────────────────────────────────────

def haversine_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calcula a distância em metros entre duas coordenadas geográficas (Haversine)."""
    R = 6371000.0  # Raio da Terra em metros
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)
    
    a = (math.sin(delta_phi / 2) ** 2 + 
         math.cos(phi1) * math.cos(phi2) * math.sin(delta_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calcula a orientação/rumo (heading) de um ponto para outro em graus (0-360)."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)
    
    y = math.sin(delta_lon) * math.cos(phi2)
    x = (math.cos(phi1) * math.sin(phi2) - 
         math.sin(phi1) * math.cos(phi2) * math.cos(delta_lon))
    
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

# ──────────────────────────────────────────────────────────────────────
# 2. GERADOR DE PAYLOAD CAM (VERSÃO CORRIGIDA)
# ──────────────────────────────────────────────────────────────────────

def generation_delta_time() -> int:
    """ETSI generationDeltaTime em millisegundos modulo 65536."""
    return int((time.time() * 1000.0) % 65536)

def build_cam_payload(
    station_id: int,
    lat: float,
    lon: float,
    speed_mps: float,
    heading_deg: float,
    *,
    station_type: int = 5,
    position_confidence_ellipse: Tuple[int, int, int] = (4095, 4095, 3601),
    altitude_value: int = 800001,
    altitude_confidence: int = 15,
    vehicle_length_value: int = 1023,
    vehicle_length_confidence_indication: int = 4,
    vehicle_width: int = 62,
    heading_confidence: int = 127,
    speed_confidence: int = 127,
    longitudinal_acceleration_value: float = 0.0,
    longitudinal_acceleration_confidence: int = 102,
    curvature_value: int = 1023,
    curvature_confidence: int = 7,
    yaw_rate_value: float = 0.0,
    yaw_rate_confidence: int = 8,
) -> dict:
    """
    VERSÃO CORRIGIDA: Build a CAM JSON compatible with Vanetza input.
    
    IMPORTANTE:
    - lat/lon são enviados como FLOATS BRUTOS (NÃO escalados)
    - O Vanetza recebe floats no /in/cam e faz a conversão interna para ASN.1
    - generationDeltaTime DENTRO de camParameters (não fora!)
    """
    
    # Enviar floats brutos, NÃO inteiros escalados
    print(f"[DEBUG] Construindo CAM: lat={lat}, lon={lon}, speed_mps={speed_mps}, heading_deg={heading_deg}")
    
    return {
        "camParameters": {
            "basicContainer": {
                "stationType": station_type,
                "referencePosition": {
                    "latitude": round(lat, 6),      # Float bruto (ex: 41.726213)
                    "longitude": round(lon, 6),     # Float bruto (ex: -8.164577)
                    "positionConfidenceEllipse": {
                        "semiMajorAxisLength": position_confidence_ellipse[0],
                        "semiMinorAxisLength": position_confidence_ellipse[1],
                        "semiMajorAxisOrientation": position_confidence_ellipse[2],
                    },
                    "altitude": {
                        "altitudeValue": altitude_value,
                        "altitudeConfidence": altitude_confidence,
                    },
                },
            },
            "highFrequencyContainer": {
                "basicVehicleContainerHighFrequency": {
                    "heading": {
                        "headingValue": round(heading_deg * 10) % 3601,  # Graus × 10
                        "headingConfidence": heading_confidence,
                    },
                    "speed": {
                        "speedValue": max(0, int(speed_mps * 100)),     # m/s × 100 (cm/s)
                        "speedConfidence": speed_confidence,
                    },
                    "driveDirection": 2,
                    "vehicleLength": {
                        "vehicleLengthValue": vehicle_length_value,
                        "vehicleLengthConfidenceIndication": vehicle_length_confidence_indication,
                    },
                    "vehicleWidth": vehicle_width,
                    "longitudinalAcceleration": {
                        "value": longitudinal_acceleration_value,
                        "confidence": longitudinal_acceleration_confidence,
                    },
                    "curvature": {
                        "curvatureValue": curvature_value,
                        "curvatureConfidence": curvature_confidence,
                    },
                    "curvatureCalculationMode": 2,
                    "yawRate": {
                        "yawRateValue": yaw_rate_value,
                        "yawRateConfidence": yaw_rate_confidence,
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
            "generationDeltaTime": generation_delta_time(),
        },
    }

# ──────────────────────────────────────────────────────────────────────
# 3. GERADOR DE PAYLOAD DENM
# ──────────────────────────────────────────────────────────────────────

def build_denm_payload(latitude: float, longitude: float, cause_code: int, sub_cause_code: int, station_type: int = 5) -> Dict[str, Any]:
    """
    Gera a estrutura exigida pelo Vanetza-NAP para alertas DENM.
    """
    
    danger_key = "wrongWayDriving14"  # Default seguro
    if cause_code == 97:
        danger_key = "collisionRisk"
    elif cause_code == 2:
        danger_key = "accident"

    return {
        "management": {
            "actionId": {
                "originatingStationId": int(time.time()) & 0xFFFFFFFF,
                "sequenceNumber": 0
            },
            "detectionTime": time.time(),
            "referenceTime": time.time(),
            "eventPosition": {
                "latitude": float(latitude),
                "longitude": float(longitude),
                "positionConfidenceEllipse": {
                    "semiMajorConfidence": 0,
                    "semiMinorConfidence": 0,
                    "semiMajorOrientation": 0
                },
                "altitude": {
                    "altitudeValue": 0,
                    "altitudeConfidence": 1
                }
            },
            "validityDuration": 60,
            "stationType": station_type
        },
        "situation": {
            "informationQuality": 7,
            "eventType": {
                "ccAndScc": {
                    danger_key: sub_cause_code
                }
            }
        }
    }
