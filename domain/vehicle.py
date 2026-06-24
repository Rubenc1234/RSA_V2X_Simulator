"""
Pure Domain Entity representing a Vehicle (C-ITS Agent).
Encapsulates kinematics, waypoint navigation, neighbor perception (CAMs),
and behavioral reaction to local hazards (DENM Registry).
"""

import math
import time
from typing import List, Tuple, Dict, Any, Optional
from domain.denm_registry import DENMRegistry

# --- Funções Geográficas Auxiliares ---
def haversine_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calcula a distância entre duas coordenadas geográficas em metros usando Haversine."""
    R = 6371000.0
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
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lon)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360


# --- Classe Principal de Domínio ---
class Vehicle:
    def __init__(self, config: Dict[str, Any]):
        # Identidade ETSI C-ITS
        self.station_id: int = config["stationId"]
        self.name: str = config["name"]
        self.vehicle_type: int = config.get("vehicleType", 5)  # 5 = Passenger Car
        
        # Contador sequencial para mensagens de rede ETSI
        self.denm_sequence_counter: int = 0
        
        # Estado Cinemático Atual (Interno em unidades standard)
        self.current_lat: float = config["startPoint"][0]
        self.current_lon: float = config["startPoint"][1]
        self.heading: float = 0.0
        self.speed_mps: float = 0.0
        
        # Controlo de Velocidade
        self.desired_speed_mps: float = config["baseSpeedMps"]
        
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

        # NOVO: Gestão de "burst" de alertas
        self.collision_burst_active: bool = False
        self.collision_burst_start_time: Optional[float] = None
        self.collision_burst_duration_s: float = 3.0  # Duração máxima do burst
        self.collision_validity_duration_s: float = 10.0  # Duração total do alerta
        self.last_collision_denm_publish_time: Optional[float] = None
        self.collision_denm_interval_s: float = 0.2  # Publicar a cada 200ms (5 Hz)

    def set_route(self, waypoints: List[Tuple[float, float]]):
        """Injeta a lista de coordenadas OSRM e reinicia o ponteiro de navegação."""
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.route_complete = False
        if waypoints:
            self.current_lat, self.current_lon = waypoints[0]

    # APIs de Controlo de Estado Superior (Invocadas pela Simuladora) ---
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

    # Construtor de Cargas Úteis de Rede ---
    def generate_collision_denm_payload(self) -> Dict[str, Any]:
        """
        Gera DENM com tempo decrementado automaticamente.
        
        Se o risco foi detectado há 2s e a duração total é 10s,
        retorna validityDuration = 8s (para sincronização automática).
        """
        self.denm_sequence_counter += 1
        now_ts = time.time()
        
        # Calcular tempo decorrido desde início da detecção
        time_elapsed = 0.0
        if self.collision_burst_start_time is not None:
            time_elapsed = now_ts - self.collision_burst_start_time
        
        # Desconto dinâmico do tempo
        remaining_validity = max(
            1.0,  # Mínimo 1s para garantir propagação
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
                "validityDuration": remaining_validity,  # ← DECREMENTADO!
                "stationType": self.vehicle_type
            },
            "situation": {
                "informationQuality": 7,
                "eventType": {
                    "ccAndScc": {"collisionRisk97": 0}
                }
            }
        }

    # --- Processamento de Sinais Recebidos da Rede (CAM) ---
    def update_neighbor_state(self, station_id: int, lat: float, lon: float, raw_speed: float, raw_heading: float):
        """Atualiza a posição de um vizinho tratando as unidades nativas da ETSI."""
        if station_id == self.station_id:
            return

        #if raw_speed == 16383 or raw_heading == 3601:
        #    return 
            
        #if lat == 900000001.0 or lon == 1800000001.0:
        #    return

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

    # Separação Limpa de Conceitos
    def step_perception(self):
        """Fase 1: Atualiza e limpa buffers de memória C-ITS."""
        self.denm_registry.clean_expired_events()
        self._clean_stale_neighbors()

    def step_decision(self, proximity_threshold: float) -> Optional[str]:
        target_speed, reason = self.evaluate_behavioral_speed(proximity_threshold)
        self.speed_mps = target_speed

        if not self.waypoints or self.route_complete:
            self.speed_mps = 0.0
            return None

        # Lógica de detecção de risco NOVO: gerenciar burst
        if reason is not None and "COLLISION_RISK" in reason:
            # Se não estava em burst, iniciar
            if not self.collision_burst_active:
                self.start_collision_burst()
            return "COLLISION_BURST_ACTIVE"  # ← Estado contínuo
        else:
            # Se estava em burst e o risco desapareceu, parar
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

    def step(self, delta_time: float, proximity_threshold: float) -> Optional[str]:
        """Orquestrador do Ciclo de Vida do Tick. Preserva compatibilidade externa."""
        self.step_perception()
        action_trigger = self.step_decision(proximity_threshold)
        self.step_physics(delta_time)
        return action_trigger

    def start_collision_burst(self):
        """Inicia estado de transmissão de alertas de colisão."""
        if not self.collision_burst_active:
            self.collision_burst_active = True
            self.collision_burst_start_time = time.time()
            self.last_collision_denm_publish_time = None  # Força publicação imediata no próximo tick
            print(f"[Vehicle {self.name}] Iniciando BURST de alertas de colisão (duração: {self.collision_burst_duration_s}s)")

    def stop_collision_burst(self):
        """Termina estado de transmissão (o evento desaparece naturalmente)."""
        if self.collision_burst_active:
            self.collision_burst_active = False
            self.collision_burst_start_time = None
            self.last_collision_denm_publish_time = None
            print(f"[Vehicle {self.name}] Parando BURST de alertas de colisão")

    def should_publish_collision_denm(self) -> bool:
        """
        Determina se é hora de publicar outro DENM de colisão.
        (respeita intervalo de 200ms entre publicações)
        """
        if not self.collision_burst_active:
            return False
        
        now = time.time()
        
        # Verificar se estamos ainda dentro da janela de burst
        burst_elapsed = now - self.collision_burst_start_time
        if burst_elapsed > self.collision_burst_duration_s:
            self.stop_collision_burst()
            return False
        
        # Verificar se já passaram 200ms desde a última publicação
        if self.last_collision_denm_publish_time is None:
            return True  # Primeira mensagem, publica já
        
        time_since_last_publish = now - self.last_collision_denm_publish_time
        if time_since_last_publish >= self.collision_denm_interval_s:
            return True
        
        return False

    def mark_collision_denm_published(self):
        """Registar que publicámos um DENM (para respeitar intervalo)."""
        self.last_collision_denm_publish_time = time.time()
