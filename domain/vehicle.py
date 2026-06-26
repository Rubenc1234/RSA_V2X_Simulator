"""
Pure Domain Entity representing a Vehicle (C-ITS Agent).
Encapsulates kinematics, waypoint navigation, neighbor perception (CAMs),
and behavioral reaction to local hazards (DENM Registry).
"""

import math
import time
from typing import List, Tuple, Dict, Any, Optional
from domain.denm_registry import DENMRegistry
from domain.utils import haversine_meters, calculate_bearing

# --- Classe Principal de Domínio ---
class Vehicle:
    def __init__(self, config: Dict[str, Any] = None, name: str = None, station_id: int = None, start_point: List[float] = None, end_point: List[float] = None, base_speed: float = 0.0, vehicle_type: int = 5):
        """
        Construtor polimórfico adaptado. Suporta dicionário de configuração completo 
        ou argumentos nomeados diretos para inicialização flexível de clones vizinhos.
        """
        if config is not None:
            self.station_id = config["stationId"]
            self.name = config["name"]
            self.vehicle_type = config.get("vehicleType", 5)  # 5 = Passenger Car
            start_pt = config["startPoint"]
            self.desired_speed_mps = config["baseSpeedMps"]
        else:
            self.station_id = station_id
            self.name = name
            self.vehicle_type = vehicle_type
            start_pt = start_point if start_point is not None else [0.0, 0.0]
            self.desired_speed_mps = base_speed
        
        # Contador sequencial para mensagens de rede ETSI
        self.denm_sequence_counter: int = 0
        
        # Estado Cinemático Atual (Interno em unidades standard)
        self.current_lat: float = start_pt[0]
        self.current_lon: float = start_pt[1]
        self.heading: float = 0.0
        self.speed_mps: float = 0.0
        
        # Rota e Navegação
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_idx: int = 0
        self.route_complete: bool = False
        
        # Sub-componentes de Percepção e Estado V2X
        self.denm_registry = DENMRegistry()
        self.perceived_neighbors: Dict[int, Dict[str, Any]] = {}
        
        # Estado Crítico de Simulação (Cenário de acidente)
        self.is_immobilized_by_accident: bool = False
        self.accident_origin_station: Optional[int] = None

        # Gestão de "burst" de alertas baseados em tempo lógico
        self.collision_burst_active: bool = False
        self.collision_burst_start_time: Optional[float] = None
        self.collision_burst_duration_s: float = 3.0  
        self.collision_validity_duration_s: float = 10.0  
        self.last_collision_denm_publish_time: Optional[float] = None
        self.collision_denm_interval_s: float = 0.2  # 5 Hz

    def set_route(self, waypoints: List[Tuple[float, float]]):
        """Injeta a lista de coordenadas OSRM e reinicia o ponteiro de navegação."""
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.route_complete = False
        if waypoints:
            self.current_lat, self.current_lon = waypoints[0]

    def immobilize_for_accident(self, origin_station_id: int = None):
        """Marca este veículo como imobilizado num cenário de acidente forçado."""
        self.is_immobilized_by_accident = True
        self.accident_origin_station = origin_station_id if origin_station_id is None else self.station_id
        print(f"[Vehicle {self.name}] IMOBILIZADO devido a acidente. Origem: Station {self.accident_origin_station}")

    def resolve_accident(self):
        """Remove o estado de imobilização (o acidente foi limpo ou resolvido)."""
        self.is_immobilized_by_accident = False
        self.accident_origin_station = None
        print(f"[Vehicle {self.name}] Estado de acidente RESOLVIDO. Retomando marcha.")

    def generate_collision_denm_payload(self, current_sim_time: float) -> Dict[str, Any]:
        """Gera DENM com tempo decrementado automaticamente usando o tempo lógico."""
        self.denm_sequence_counter += 1
        now_ts = current_sim_time
        
        time_elapsed = 0.0
        if self.collision_burst_start_time is not None:
            time_elapsed = now_ts - self.collision_burst_start_time
        
        remaining_validity = max(
            1.0,  # Mínimo 1s para propagação estável
            self.collision_validity_duration_s - time_elapsed
        )
        
        return {
            "management": {
                "actionId": {
                    "originatingStationId": self.station_id,
                    "sequenceNumber": self.denm_sequence_counter
                },
                "detectionTime": self.collision_burst_start_time or now_ts,
                "referenceTime": now_ts,
                "eventPosition": {
                    "latitude": self.current_lat,
                    "longitude": self.current_lon
                },
                "validityDuration": remaining_validity,
                "stationType": self.vehicle_type
            },
            "situation": {
                "informationQuality": 7,
                "eventType": {
                    "ccAndScc": {"collisionRisk97": 0}
                }
            }
        }
    
    def generate_accident_denm_payload(self, current_sim_time: float) -> Dict[str, Any]:
        """Gera DENM para acidente em tempo de simulação lógico."""
        self.denm_sequence_counter += 1
        now_ts = current_sim_time
        
        return {
            "management": {
                "actionId": {
                    "originatingStationId": self.station_id,
                    "sequenceNumber": self.denm_sequence_counter
                },
                "detectionTime": now_ts,
                "referenceTime": now_ts,
                "eventPosition": {
                    "latitude": self.current_lat,
                    "longitude": self.current_lon
                },
                "validityDuration": 60.0,
                "stationType": self.vehicle_type
            },
            "situation": {
                "informationQuality": 7,
                "eventType": {
                    "ccAndScc": {"accident2": 0}
                }
            }
        }

    def update_neighbor_state(self, station_id: int, lat: float, lon: float, raw_speed: float, raw_heading: float):
        """Atualiza a posição de um vizinho tratando as unidades nativas da ETSI."""
        if station_id == self.station_id:
            return

        converted_speed = raw_speed * 0.01
        converted_heading = raw_heading * 0.1

        self.perceived_neighbors[station_id] = {
            "lat": lat,
            "lon": lon,
            "speed": converted_speed,
            "heading": converted_heading,
            "timestamp": time.time()
        }

    def _clean_stale_neighbors(self, max_age_seconds: float = 3.0):
        """Remove vizinhos que saíram do alcance de rádio."""
        now = time.time()
        stale_keys = [k for k, v in self.perceived_neighbors.items() if now - v["timestamp"] > max_age_seconds]
        for k in stale_keys:
            del self.perceived_neighbors[k]

    def evaluate_behavioral_speed(self, proximity_threshold: float) -> Tuple[float, Optional[str]]:
        """Avalia dinamicamente os perigos locais para ditar a velocidade ideal."""
        if self.is_immobilized_by_accident:
            return 0.0, "ACCIDENT_ISSUER"

        for event_key, event in self.denm_registry.active_events.items():
            cc_and_scc = event.get("ccAndScc", {})
            if "accident2" in cc_and_scc:
                event_lat, event_lon = event["position"]
                if event_lat is None or event_lon is None:
                    continue
                    
                dist_to_accident = haversine_meters(self.current_lat, self.current_lon, event_lat, event_lon)
                if dist_to_accident < 60.0:
                    return 0.0, f"AHEAD_ACCIDENT_ALERT_ID_{event_key}"

        for n_id, neighbor in self.perceived_neighbors.items():
            dist = haversine_meters(self.current_lat, self.current_lon, neighbor["lat"], neighbor["lon"])
            if dist < proximity_threshold:
                if dist < 15.0:
                    return 0.0, f"EMERGENCY_BRAKE_COLLISION_RISK_WITH_{n_id}"
                else:
                    return self.desired_speed_mps * 0.4, f"SLOW_DOWN_PROXIMITY_WITH_{n_id}"

        return self.desired_speed_mps, None

    def step_perception(self):
        """Fase 1: Atualiza e limpa buffers de memória C-ITS."""
        self.denm_registry.clean_expired_events()
        self._clean_stale_neighbors()

    def step_decision(self, proximity_threshold: float, current_sim_time: float) -> Optional[str]:
        """Fase 2: Avaliação comportamental e controlo de rajadas (Burst)."""
        target_speed, reason = self.evaluate_behavioral_speed(proximity_threshold)
        self.speed_mps = target_speed

        if not self.waypoints or self.route_complete:
            self.speed_mps = 0.0
            return None

        if reason is not None and "COLLISION_RISK" in reason:
            if not self.collision_burst_active:
                self.start_collision_burst(current_sim_time)
            return "COLLISION_BURST_ACTIVE"
        else:
            if self.collision_burst_active:
                self.stop_collision_burst()
        
        return None

    def step_physics(self, delta_time: float):
        """Fase 3: Executa as equações de movimento linear na rota OSRM."""
        if self.route_complete or self.speed_mps == 0.0:
            return

        distance_to_travel = self.speed_mps * delta_time
        
        while distance_to_travel > 0 and not self.route_complete:
            next_wp = self.waypoints[self.current_waypoint_idx]
            dist_to_next = haversine_meters(self.current_lat, self.current_lon, next_wp[0], next_wp[1])
            
            if dist_to_next <= distance_to_travel:
                self.current_lat, self.current_lon = next_wp
                distance_to_travel -= dist_to_next
                self.current_waypoint_idx += 1
                
                if self.current_waypoint_idx >= len(self.waypoints):
                    self.route_complete = True
            else:
                ratio = distance_to_travel / dist_to_next
                self.heading = calculate_bearing(self.current_lat, self.current_lon, next_wp[0], next_wp[1])
                
                self.current_lat += (next_wp[0] - self.current_lat) * ratio
                self.current_lon += (next_wp[1] - self.current_lon) * ratio
                distance_to_travel = 0.0
                
                self.heading = calculate_bearing(self.current_lat, self.current_lon, next_wp[0], next_wp[1])

    def step(self, delta_time: float, proximity_threshold: float, current_sim_time: float) -> Optional[str]:
        """Orquestrador do Ciclo de Vida do Tick. Mantém compatibilidade externa."""
        self.step_perception()
        action_trigger = self.step_decision(proximity_threshold, current_sim_time)
        self.step_physics(delta_time)
        return action_trigger

    def start_collision_burst(self, current_sim_time: float):
        """Inicia o estado de transmissão de rajada de alertas de colisão usando o tempo lógico."""
        if not self.collision_burst_active:
            self.collision_burst_active = True
            self.collision_burst_start_time = current_sim_time
            self.last_collision_denm_publish_time = None  
            print(f"[Vehicle {self.name}] Iniciando BURST de alertas de colisão (duração: {self.collision_burst_duration_s}s)")

    def stop_collision_burst(self):
        """Termina estado de transmissão."""
        if self.collision_burst_active:
            self.collision_burst_active = False
            self.collision_burst_start_time = None
            self.last_collision_denm_publish_time = None
            print(f"[Vehicle {self.name}] Parando BURST de alertas de colisão")

    def should_publish_collision_denm(self, current_sim_time: float) -> bool:
        if not self.collision_burst_active:
            return False
        
        burst_elapsed = current_sim_time - self.collision_burst_start_time
        if burst_elapsed > self.collision_burst_duration_s:
            self.stop_collision_burst()
            return False
        
        if self.last_collision_denm_publish_time is None:
            return True
        
        time_since_last_publish = current_sim_time - self.last_collision_denm_publish_time
        return time_since_last_publish >= self.collision_denm_interval_s

    def mark_collision_denm_published(self, current_sim_time: float):
        """Registar que publicámos um DENM no tempo lógico atual."""
        self.last_collision_denm_publish_time = current_sim_time
