# vehicle_roles.py
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

# Alinhado com as constantes do teu simulator_core
DENM_CAUSE_ACCIDENT = 2

class VehicleRole(str, Enum):
    NORMAL = "normal"
    EMERGENCY = "emergency"
    EMERGENCY_REACTOR = "emergency_reactor"

@dataclass
class CollisionBehavior:
    enabled: bool = True
    emit_collision_risk_denm: bool = True
    denm_stop_distance_m: float = 110.0
    denm_slowdown_distance_m: float = 180.0
    denm_ignore_distance_m: float = 320.0

@dataclass
class AccidentBehavior:
    hard_stop_on_accident: bool = True
    reroute_on_accident: bool = False
    reroute_prefer_alternative: bool = True
    accident_stop_distance_m: float = 140.0
    accident_ignore_distance_m: float = 260.0
    incident_on_arrival: bool = False
    incident_after_seconds: Optional[float] = None
    incident_cause_code: int = DENM_CAUSE_ACCIDENT
    incident_sub_cause_code: int = 0
    incident_validity_duration: int = 10
    incident_reset_seconds: float = 12.0

@dataclass
class CorridorBehavior:
    lane: Optional[str] = None          # "left" | "right" | "random"
    lane_offset_m: float = 4.5
    start_offset_m: float = 0.0
    
    # Campos que existiam no car_agent.py mapeados de forma exata:
    emergency_denm_interval_s: float = 1.0
    emergency_denm_validity_s: int = 3
    emergency_denm_sub_cause: int = 1
    yield_distance_threshold_m: float = 180.0
    yield_duration_s: float = 6.0
    
    # Definido dinamicamente via fábrica para evitar dupla fonte de verdade
    is_emergency: bool = field(init=False, default=False)

    @classmethod
    def from_profile(cls, profile: dict, role: VehicleRole) -> Optional['CorridorBehavior']:
        """
        Fábrica que extrai as configurações do perfil e injeta a propriedade 
        'is_emergency' baseada estritamente no papel (role) atribuído.
        """
        if role not in (VehicleRole.EMERGENCY, VehicleRole.EMERGENCY_REACTOR):
            return None
        
        corridor_data = profile.get("corridor", {})
        
        # Cria a instância limpando possíveis injeções manuais de is_emergency do dict
        instance = cls(**corridor_data)
        
        # A única fonte de verdade dita se é um veículo de emergência ativa ou não:
        instance.is_emergency = (role == VehicleRole.EMERGENCY)
        
        return instance
