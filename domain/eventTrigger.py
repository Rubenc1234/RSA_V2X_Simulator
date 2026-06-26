"""
Motor de Eventos Dinâmicos para o Simulador V2X.
Isolado para evitar dependências circulares e permitir reutilização.
"""

from __future__ import annotations
import time
from typing import Dict, List, Any, Optional

from domain.vehicle import Vehicle


class EventTrigger:
    """Define um evento configurável no cenário (colisão, acidente, etc)."""
    
    def __init__(self, event_config: Dict[str, Any]):
        self.event_id = event_config["event_id"]          # Ex: "col_1", "acc_2"
        self.event_type = event_config["event_type"]      # Ex: "COLLISION_RISK", "ACCIDENT"
        self.trigger_condition = event_config.get("trigger_condition")  # lambda ou callable
        self.trigger_position = event_config.get("trigger_position")    # (lat, lon) ou None
        self.affected_vehicles = event_config["affected_vehicles"]      # ["obu1", "obu2"]
        self.severity = event_config.get("severity", "medium")
        self.is_active = False
        self.activated_at: Optional[float] = None
        
    def evaluate(self, vehicles: Dict[str, Vehicle]) -> bool:
        """Verifica se as condições do ambiente dão gatilho ao evento."""
        if self.is_active:
            return True
            
        if callable(self.trigger_condition):
            if self.trigger_condition(vehicles):
                self.is_active = True
                self.activated_at = time.time()
                return True
        return False


class EventTriggerEngine:
    """Gere a coleção de gatilhos de eventos e avalia o estado do cenário a cada tick."""
    
    def __init__(self, events_config: List[Dict[str, Any]]):
        self.events: List[EventTrigger] = [EventTrigger(cfg) for cfg in events_config]
        
    def check_and_update(self, vehicles: Dict[str, Vehicle]) -> List[EventTrigger]:
        """Avalia todos os eventos e retorna a lista de eventos atualmente ativos."""
        active_events = []
        for event in self.events:
            if event.evaluate(vehicles):
                active_events.append(event)
        return active_events
