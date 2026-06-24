from typing import Dict, Any

# Configurações globais de segurança válidas para QUALQUER cenário
GLOBAL_SETTINGS = {
    "proximity_threshold_meters": 45.0,  # Distância para deteção de risco de colisão
    "tick_hz": 5.0,                      # Frequência da simulação
    "etsi_cause_accident": 2,            # ETSI Cause Code para Acidente
    "etsi_cause_collision_risk": 97,     # ETSI Cause Code para Risco de Colisão
}

SCENARIOS: Dict[str, Any] = {
    "collision_risk": {
        "name": "collision_risk",
        "label": "Cenário 1: Risco de Colisão (Aproximação Natural)",
        "description": "Veículos são colocados em trajetórias opostas. A deteção de risco ocorre dinamicamente via CAMs.",
        "mapCenter": [41.726350, -8.164850],  # Braga, R. Arnaçó
        "mapZoom": 17,
        "brokers": [
            {"host": "192.168.98.20", "name": "obu1", "stationId": 2},
            {"host": "192.168.98.21", "name": "obu2", "stationId": 3},
        ],
        "vehicles": [
            {
                "name": "obu1",
                "stationId": 2,
                "broker": "192.168.98.20",
                "startPoint": [41.726200, -8.164600],
                "endPoint": [41.726800, -8.165200],
                "baseSpeedMps": 11.1,  # ~40 km/h
                "vehicleType": 5       # Passenger car
            },
            {
                "name": "obu2",
                "stationId": 3,
                "broker": "192.168.98.21",
                "startPoint": [41.726800, -8.165200],
                "endPoint": [41.726200, -8.164600],
                "baseSpeedMps": 8.3,   # ~30 km/h
                "vehicleType": 5
            }
        ]
    },
    
    "accident": {
        "name": "accident",
        "label": "Cenário 2: Acidente Rodoviário e Negação/Cancelamento",
        "description": "OBU1 imobiliza-se artificialmente e emite uma DENM de Acidente. OBU2 reage. Mais tarde, é emitida a negação.",
        "mapCenter": [41.160400, -8.629800],  # Porto
        "mapZoom": 17,
        "brokers": [
            {"host": "192.168.98.20", "name": "obu1", "stationId": 2},
            {"host": "192.168.98.21", "name": "obu2", "stationId": 3},
        ],
        "vehicles": [
            {
                "name": "obu1",
                "stationId": 2,
                "broker": "192.168.98.20",
                "startPoint": [41.160692, -8.628405],
                "endPoint": [41.160100, -8.631100],
                "baseSpeedMps": 10.0,
                "vehicleType": 5
            },
            {
                "name": "obu2",
                "stationId": 3,
                "broker": "192.168.98.21",
                "startPoint": [41.159900, -8.628900],
                "endPoint": [41.160800, -8.630600],
                "baseSpeedMps": 10.0,
                "vehicleType": 5
            }
        ],
        "event_timeline": {
            "accident_trigger_tick": 25,       # Tick em que o acidente é forçado na OBU1 (5s * 5Hz = 25 ticks)
            "accident_vehicle": "obu1",
            "resolution_delay_seconds": 20.0,  # Tempo até enviar a negação/cancelamento (20s após o acidente)
            "validity_duration_seconds": 60    # Validade inicial da DENM
        }
    }
}

def get_scenario_config(scenario_name: str) -> Dict[str, Any]:
    if scenario_name not in SCENARIOS:
        raise ValueError(f"Cenário '{scenario_name}' inválido.")
    return SCENARIOS[scenario_name]

def get_vehicle_config(scenario_name: str, vehicle_name: str) -> Dict[str, Any]:
    scenario = get_scenario_config(scenario_name)
    for vehicle in scenario["vehicles"]:
        if vehicle["name"] == vehicle_name:
            return vehicle
    raise ValueError(f"Veículo '{vehicle_name}' não existe no cenário '{scenario_name}'.")
