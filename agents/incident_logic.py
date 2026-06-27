"""Module for handling accident and incident lifecycle logic.

Functions receive the 'agent' instance as their first argument to manipulate
and maintain state within the central CarAgent.
"""
from __future__ import annotations

import time
from types import SimpleNamespace

import simulator_core as core
from agents.vehicle_roles import AccidentBehavior


def trigger_accident_denm(agent, reason: str) -> None:
    """Triggers an accident state on the agent, halts the vehicle simulation,

    and publishes the corresponding DENM message to the network.
    """
    if agent.incident_triggered:
        return

    agent.incident_triggered = True
    agent.incident_triggered_at = time.time()
    agent.vehicle.hard_stop = True
    agent.vehicle.current_speed_mps = 0.0
    agent.vehicle.target_speed_mps = 0.0
    agent.avoidance_active = True
    agent.yield_vehicle_name = agent.vehicle.name
    agent.yield_mode = "stop"
    agent.denm_hold_until = time.time() + 9999.0

    event_position = (agent.vehicle.current_lat, agent.vehicle.current_lon)
    print(f"[{agent.name}] INCIDENTE {reason} -> veículo parado e DENM de acidente emitido")

    notify = [SimpleNamespace(client=agent.vehicle.client)]
    if agent.peer_clients:
        notify.extend(SimpleNamespace(client=client) for client in agent.peer_clients)

    core.publish_denm(
        agent.vehicle,
        cause_code=int(agent.accident.incident_cause_code),
        sub_cause_code=str(agent.accident.incident_sub_cause_code) if agent.accident.incident_sub_cause_code else None,
        validity_duration=agent.accident.incident_validity_duration,
        event_position=event_position,
        notify_vehicles=notify,
    )


def reset_after_accident(agent) -> None:
    """Resets the accident state of the agent, releasing the vehicle to resume

    and restart its route parameters.
    """
    agent.vehicle.hard_stop = False
    agent.vehicle._restart_route()
    agent.vehicle.current_speed_mps = agent.vehicle.base_speed_mps
    agent.vehicle.target_speed_mps = agent.vehicle.base_speed_mps
    agent.avoidance_active = False
    agent.yield_vehicle_name = ""
    agent.yield_mode = "stop"
    agent.denm_hold_until = 0.0
    agent.incident_triggered = False
    agent.incident_triggered_at = 0.0
    print(f"[{agent.name}] ACCIDENT RESET -> rota reiniciada e veículo pronto para repetir")


def maybe_reset_after_accident(agent) -> None:
    """Evaluates if the accident recovery timer has elapsed, triggering a reset

    if the required duration has passed.
    """
    if not agent.vehicle.hard_stop or not agent.incident_triggered:
        return
    if agent.incident_triggered_at <= 0.0:
        return
    if (time.time() - agent.incident_triggered_at) >= agent.accident.incident_reset_seconds:
        reset_after_accident(agent)
