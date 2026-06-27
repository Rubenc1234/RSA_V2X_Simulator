"""Configuration constants for V2X simulator.

All magic numbers and tunable parameters go here to avoid hardcoding.
"""

# ═══════════════════════════════════════════════════════════════════
# ACCIDENT/INCIDENT BEHAVIOR
# ═══════════════════════════════════════════════════════════════════

# How long the accident DENM stays valid in the network
INCIDENT_VALIDITY_DURATION_S = 10.0

# How long until vehicle can reset from accident state
INCIDENT_RESET_SECONDS = 12.0

# How long agent holds accident state before accepting new events
INCIDENT_HOLD_UNTIL_S = 10.0  # ✅ REPLACES hardcoded 9999.0

# ═══════════════════════════════════════════════════════════════════
# COLLISION DETECTION
# ═══════════════════════════════════════════════════════════════════

# Distance threshold for publishing collision risk DENM
COLLISION_WARNING_DISTANCE_M = 80.0

# Distance threshold for clearing collision risk
COLLISION_CLEAR_DISTANCE_M = 100.0

# Speed reduction factor when in collision risk (0.3 = 30%)
COLLISION_SLOWDOWN_FACTOR = 0.3
COLLISION_SLOWDOWN_DURATION_S = 3.0

# ═══════════════════════════════════════════════════════════════════
# AVOIDANCE STATE MANAGEMENT
# ═══════════════════════════════════════════════════════════════════

# Priority levels for decision-making
AVOIDANCE_PRIORITY = {
    "accident": 3,
    "collision_risk": 2,
    "soft_yield": 1,
    "none": 0,
}

# ═══════════════════════════════════════════════════════════════════
# CORRIDOR (EMERGENCY) BEHAVIOR
# ═══════════════════════════════════════════════════════════════════

# Interval between emergency DENMs
CORRIDOR_DENM_INTERVAL_S = 1.0
CORRIDOR_DENM_VALIDITY_S = 3
CORRIDOR_DENM_SUB_CAUSE = 1

# Distance at which normal vehicles yield to emergency vehicles
CORRIDOR_YIELD_DISTANCE_M = 180.0
CORRIDOR_YIELD_DURATION_S = 6.0

# Lane offset for emergency corridor
CORRIDOR_LANE_OFFSET_M = 4.5
