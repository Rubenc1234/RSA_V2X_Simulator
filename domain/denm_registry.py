import json
from typing import Dict, Any, Optional

from domain.vehicle import haversine_meters

class DENMRegistry:
    def __init__(self):
        # Guarda os eventos ativos
        self.active_events: Dict[str, Dict[str, Any]] = {}
        # Guarda o último tempo conhecido do simulador para limpezas offline
        self.latest_sim_time: float = 0.0
    
    def _generate_key(self, action_id: Dict[str, Any]) -> str:
        """Gera uma chave única string a partir do actionId do ETSI Vanetza."""
        station_id = action_id.get("originatingStationId")
        seq_num = action_id.get("sequenceNumber")
        return f"{station_id}:{seq_num}"

    def register_denm_json(self, json_str: str) -> Optional[str]:
        """
        Consome o JSON real do vanetza/out/denm e regista-o usando o tempo lógico.
        """
        try:
            payload = json.loads(json_str)
            
            # 1. Extrair o tempo da simulação vindo da raiz do JSON
            msg_timestamp = payload.get("timestamp")
            if msg_timestamp and msg_timestamp > self.latest_sim_time:
                self.latest_sim_time = msg_timestamp

            fields = payload.get("fields", {})
            denm = fields.get("denm", {})
            management = denm.get("management", {})
            situation = denm.get("situation", {})
            
            # 2. Identificador único da ETSI (StationID + SequenceNumber)
            action_id = management.get("actionId", {})
            orig_station = action_id.get("originatingStationId")
            seq_num = action_id.get("sequenceNumber")
            
            if orig_station is None or seq_num is None:
                return "INVALID_ACTION_ID"
                
            event_key = f"{orig_station}:{seq_num}"
            
            # 3. Calcular expiração com base no tempo do simulador
            detection_time = management.get("detectionTime", msg_timestamp)
            validity_duration = management.get("validityDuration", 0)
            expire_at = detection_time + validity_duration
            
            # Se a mensagem já chegou expirada no tempo virtual, ignora
            if msg_timestamp and msg_timestamp > expire_at:
                return "EXPIRED_ON_ARRIVAL"
                
            # 4. Extrair Posição Geo
            event_pos = management.get("eventPosition", {})
            lat = event_pos.get("latitude")
            lon = event_pos.get("longitude")
            
            # 5. Extrair contentor de causas (ex: {"accident2": 0})
            cc_and_scc = situation.get("eventType", {}).get("ccAndScc", {})
            
            # Guardar no dicionário local
            self.active_events[event_key] = {
                "position": (lat, lon),
                "ccAndScc": cc_and_scc,
                "expire_at": expire_at
            }
            
            print(f"[DENM Registry] Alerta atualizado. Chave: {event_key} | Causas: {list(cc_and_scc.keys())}")
            return "SUCCESS"

        except Exception as e:
            print(f"[DENM Registry] Erro crítico ao processar payload: {e}")
            return "PARSE_ERROR"

    def clean_expired_events(self):
        """
        Limpa eventos cujo tempo de validade expirou face ao tempo lógico da simulação.
        """
        if self.latest_sim_time == 0.0:
            return # Ainda não recebemos mensagens para balizar o tempo
            
        expired_keys = [
            k for k, v in self.active_events.items() 
            if self.latest_sim_time > v["expire_at"]
        ]
        
        for k in expired_keys:
            print(f"[DENM Registry] Evento {k} atingiu o fim da validade e foi removido.")
            del self.active_events[k]
    
    def find_nearby_danger(self, current_lat: float, current_lon: float, max_distance_meters: float, danger_type: str) -> Optional[str]:
        """
        Procura um perigo específico dentro de um raio.
        Retorna o ID do evento se encontrar.
        """

        for event_id, event in self.active_events.items():

            cc_and_scc = event.get("ccAndScc", {})

            # Verifica o tipo ETSI
            if danger_type not in cc_and_scc:
                continue

            event_lat, event_lon = event["position"]

            if event_lat is None or event_lon is None:
                continue

            distance = haversine_meters(
                current_lat,
                current_lon,
                event_lat,
                event_lon
            )

            if distance <= max_distance_meters:
                return event_id

        return None
