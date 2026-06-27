# avoidance_state.py (novo módulo pequeno)
import time

PRIORITY = {
    "accident": 3,
    "collision_risk": 2,
    "soft_yield": 1,
    "none": 0,
}

def request_avoidance(agent, mode: str, speed_factor: float, hold_s: float, am_i_yielding: bool) -> bool:
    """Pede para entrar em modo de evasão. Só é aceite se tiver prioridade
    suficiente ou se o estado atual já tiver expirado. Devolve True se aceite.
    """
    now = time.time()
    current_priority = PRIORITY.get(agent.yield_mode, 0)
    new_priority = PRIORITY.get(mode, 0)
    expired = now >= agent.denm_hold_until

    if not (expired or new_priority >= current_priority):
        return False  # estado atual mais importante e ainda válido -> ignora pedido

    agent.yield_mode = mode
    agent.avoidance_active = True
    agent.denm_hold_until = now + hold_s

    factor = speed_factor if am_i_yielding else min(1.0, speed_factor + 0.25)
    agent.vehicle.target_speed_mps = max(0.5, agent.vehicle.base_speed_mps * factor)
    return True


def clear_if_expired(agent) -> None:
    """Chamado uma vez por tick (no loop principal) para libertar o veículo
    quando o hold expira e nada novo o renovou."""
    if agent.avoidance_active and time.time() >= agent.denm_hold_until:
        agent.avoidance_active = False
        agent.yield_mode = "none"
        agent.yield_vehicle_name = ""
        agent.vehicle.target_speed_mps = agent.vehicle.base_speed_mps
