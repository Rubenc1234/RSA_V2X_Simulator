# vehicle_roles.py
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from agents.config import (
    INCIDENT_VALIDITY_DURATION_S,
    INCIDENT_RESET_SECONDS,
    CORRIDOR_DENM_INTERVAL_S,
    CORRIDOR_DENM_VALIDITY_S,
    CORRIDOR_DENM_SUB_CAUSE,
    CORRIDOR_YIELD_DISTANCE_M,
    CORRIDOR_YIELD_DURATION_S,
    CORRIDOR_LANE_OFFSET_M,
)

# Alinhado com as constantes do teu simulator_core
DENM_CAUSE_ACCIDENT = 2


class VehicleRole(str, Enum):
    NORMAL = "normal"
    EMERGENCY = "emergency"
    EMERGENCY_REACTOR = "emergency_reactor"


@dataclass
class CollisionBehavior:
    """Configuration for collision detection and reaction behavior."""
    enabled: bool = True
    emit_collision_risk_denm: bool = True
    denm_stop_distance_m: float = 110.0
    denm_slowdown_distance_m: float = 180.0
    denm_ignore_distance_m: float = 320.0


@dataclass
class AccidentBehavior:
    """Configuration for accident/incident lifecycle management."""
    hard_stop_on_accident: bool = True
    reroute_on_accident: bool = False
    reroute_prefer_alternative: bool = True
    accident_stop_distance_m: float = 140.0
    accident_ignore_distance_m: float = 260.0
    incident_on_arrival: bool = False
    incident_after_seconds: Optional[float] = None
    incident_cause_code: int = DENM_CAUSE_ACCIDENT
    incident_sub_cause_code: int = 0
    incident_validity_duration: int = INCIDENT_VALIDITY_DURATION_S
    incident_reset_seconds: float = INCIDENT_RESET_SECONDS


@dataclass
class CorridorBehavior:
    """Configuration for emergency corridor operations and lane changes."""
    lane: Optional[str] = None          # "left" | "right" | None
    lane_offset_m: float = CORRIDOR_LANE_OFFSET_M
    start_offset_m: float = 0.0
    
    # Emergency DENM publishing
    emergency_denm_interval_s: float = CORRIDOR_DENM_INTERVAL_S
    emergency_denm_validity_s: int = CORRIDOR_DENM_VALIDITY_S
    emergency_denm_sub_cause: int = CORRIDOR_DENM_SUB_CAUSE
    
    # Yield behavior for non-emergency vehicles reacting to emergency
    yield_distance_threshold_m: float = CORRIDOR_YIELD_DISTANCE_M
    yield_duration_s: float = CORRIDOR_YIELD_DURATION_S
    
    # Injected dynamically via from_profile factory
    is_emergency: bool = field(init=False, default=False)

    @classmethod
    def from_profile(cls, profile: dict, role: VehicleRole) -> Optional['CorridorBehavior']:
        """
        Factory method that extracts corridor configuration from vehicle profile
        and injects the 'is_emergency' property based strictly on the assigned role.
        
        Returns None if the role is not EMERGENCY or EMERGENCY_REACTOR.
        """
        if role not in (VehicleRole.EMERGENCY, VehicleRole.EMERGENCY_REACTOR):
            return None
        
        corridor_data = profile.get("corridor", {})
        
        # Create instance from profile data
        instance = cls(**corridor_data)
        
        # Single source of truth: is_emergency depends only on role
        instance.is_emergency = (role == VehicleRole.EMERGENCY)
        
        return instance
