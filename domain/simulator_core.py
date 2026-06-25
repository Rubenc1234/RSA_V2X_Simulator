"""
Integra a movimentação cinemática, rede MQTT e o Motor de Eventos Dinâmicos (EventTrigger).
"""

from __future__ import annotations

import json
import time
from typing import Dict, List, Tuple, Any, Optional

import paho.mqtt.client as mqtt
import requests

# Importações do domínio do projeto
from scenarios.registry import get_scenario_config
from domain.vehicle import Vehicle, haversine_meters

OSRM_SERVER = "http://router.project-osrm.org"


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
        if callable(self.trigger_condition):
            try:
                return self.trigger_condition(vehicles)
            except Exception as e:
                print(f"[EventTrigger Error] Falha ao avaliar lambda do evento {self.event_id}: {e}")
                return False
        return False
        
    def on_activate(self, sim_time: float) -> None:
        """Chamado no instante em que o evento dispara."""
        self.is_active = True
        self.activated_at = sim_time
        print(f"\n [EventTrigger] {self.event_id} ({self.event_type}) ATIVADO no tempo {sim_time:.1f}s")
        
    def on_deactivate(self) -> None:
        """Chamado quando as condições do evento deixam de ser verdade."""
        self.is_active = False
        print(f" [EventTrigger] {self.event_id} DESATIVADO / RESOLVIDO")


class EventTriggerEngine:
    """Orquestrador que avalia a linha de eventos do cenário a cada tick."""
    
    def __init__(self, scenario_config: Dict[str, Any]):
        self.events: Dict[str, EventTrigger] = {}
        
        # Carrega e instancia os objetos de gatilho declarados na configuração
        for event_config in scenario_config.get("events", []):
            event = EventTrigger(event_config)
            self.events[event.event_id] = event
    
    def update(self, vehicles: Dict[str, Vehicle], sim_time: float) -> List[EventTrigger]:
        """
        Avalia as condições de todos os eventos. 
        Retorna uma lista de triggers que acabaram de transitar para ATIVO.
        """
        newly_activated_events = []
        
        for event in self.events.values():
            was_active = event.is_active
            is_active = event.evaluate(vehicles)
            
            if is_active and not was_active:
                event.on_activate(sim_time)
                newly_activated_events.append(event)
            elif not is_active and was_active:
                event.on_deactivate()
                
        return newly_activated_events


class SimulatorCore:
    def __init__(self, scenario_name: str, mqtt_broker_hosts: Optional[Dict[str, str]] = None):
        """ Inicializa o motor do simulador. """
        self.scenario_name = scenario_name
        self.scenario_config = get_scenario_config(scenario_name)
        
        self.vehicles: Dict[str, Vehicle] = {}
        self.mqtt_clients: Dict[str, mqtt.Client] = {}
        
        self.tick_count: int = 0
        self.simulation_time: float = 0.0
        self.delta_time: float = 0.2  # 200ms -> Frequência de 5 Hz
        
        # Injeta o motor de eventos extra fornecido
        self.event_engine = EventTriggerEngine(self.scenario_config)

    def _fetch_osrm_route(self, start_point: List[float], end_point: List[float]) -> List[Tuple[float, float]]:
        url = f"{OSRM_SERVER}/route/v1/driving/{start_point[1]},{start_point[0]};{end_point[1]},{end_point[0]}?overview=full&geometries=geojson"
        try:
            response = requests.get(url, timeout=5)
            if response.status_code == 200:
                data = response.json()
                if "routes" in data and len(data["routes"]) > 0:
                    coords = data["routes"][0]["geometry"]["coordinates"]
                    return [(float(coord[1]), float(coord[0])) for coord in coords]
        except Exception as e:
            print(f"[SimulatorCore OSRM] Fallback para rota linear: {e}")
        return [(start_point[0], start_point[1]), (end_point[0], end_point[1])]

    def load_scenario(self) -> None:
        vehicles_list = self.scenario_config.get("vehicles", [])
        for v_config in vehicles_list:
            vehicle = Vehicle(v_config)
            start_pt = v_config.get("startPoint")
            end_pt = v_config.get("endPoint")
            
            if start_pt and end_pt:
                waypoints = self._fetch_osrm_route(start_pt, end_pt)
                vehicle.set_route(waypoints)
                
            self.vehicles[vehicle.name] = vehicle
            print(f"[SimulatorCore] Carregado {vehicle.name} (StationID: {vehicle.station_id})")

    def setup_mqtt(self) -> None:
        brokers_list = self.scenario_config.get("brokers", [])
        for b_config in brokers_list:
            host = b_config.get("host")
            broker_name = b_config.get("name")
            if not host or host in self.mqtt_clients:
                continue
                
            client = mqtt.Client(client_id=f"sim_core_{broker_name}")
            client.user_data_set(self)
            client.on_message = self._on_mqtt_message
            
            try:
                client.connect(host, 1883, keepalive=60)
                client.subscribe("vanetza/out/cam", qos=0)
                client.subscribe("vanetza/out/denm", qos=0)
                client.loop_start()
                self.mqtt_clients[host] = client
            except Exception as e:
                print(f"[SimulatorCore MQTT Error] Falha de rede em {host}: {e}")

    def _on_mqtt_message(self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
        try:
            topic = msg.topic
            payload_str = msg.payload.decode("utf-8")
            
            if topic == "vanetza/out/cam":
                data = json.loads(payload_str)
                station_id = data.get("stationID")
                fields = data.get("fields", {}).get("cam", {}).get("camParameters", {})
                ref_pos = fields.get("basicContainer", {}).get("referencePosition", {})
                hf = fields.get("highFrequencyContainer", {}).get("basicVehicleContainerHighFrequency", {})
                
                if station_id:
                    self.on_cam_received(
                        station_id, 
                        ref_pos.get("latitude"), 
                        ref_pos.get("longitude"), 
                        hf.get("speed", {}).get("speedValue", 0.0), 
                        hf.get("heading", {}).get("headingValue", 0.0)
                    )
            elif topic == "vanetza/out/denm":
                self.on_denm_received(payload_str)
        except Exception as e:
            pass

    def on_cam_received(self, station_id: int, lat: float, lon: float, speed: float, heading: float) -> None:
        for vehicle in self.vehicles.values():
            vehicle.update_neighbor_state(station_id, lat, lon, speed, heading)

    def on_denm_received(self, json_denm: str) -> None:
        for vehicle in self.vehicles.values():
            vehicle.denm_registry.register_denm_json(json_denm)

    def get_broker_for_vehicle(self, vehicle_name: str) -> str:
        vehicles_list = self.scenario_config.get("vehicles", [])
        for v in vehicles_list:
            if v.get("name") == vehicle_name:
                return v.get("broker", "127.0.0.1")
        return "127.0.0.1"

    # =========================================================================
    # PARTE 2: IMPLEMENTAÇÃO DO FLUXO DO PASSO 2-4 (CICLO DE TICK ACTIVO)
    # =========================================================================

    def publish_cam_for_all_vehicles(self) -> None:
        """Varre os veículos vivos e injeta os pacotes CAM na entrada do Vanetza."""
        for name, vehicle in self.vehicles.items():
            cam_payload = {
                "camParameters": {
                    "basicContainer": {
                        "stationType": vehicle.vehicle_type,
                        "referencePosition": {
                            "latitude": vehicle.current_lat,
                            "longitude": vehicle.current_lon
                        }
                    },
                    "highFrequencyContainer": {
                        "basicVehicleContainerHighFrequency": {
                            "heading": {"headingValue": float(vehicle.heading)},
                            "speed": {"speedValue": float(vehicle.speed_mps)},
                            "vehicleLength": {"vehicleLengthValue": 450}, # 4.5 metros padrão
                            "vehicleWidth": 18
                        }
                    }
                }
            }
            broker_host = self.get_broker_for_vehicle(name)
            if broker_host in self.mqtt_clients:
                self.mqtt_clients[broker_host].publish("vanetza/in/cam", json.dumps(cam_payload), qos=0)

    def publish_denm(self, vehicle_name: str, denm_payload: Dict[str, Any]) -> None:
        """Publica um evento estruturado de perigo no broker atribuído à OBU."""
        broker_host = self.get_broker_for_vehicle(vehicle_name)
        if broker_host in self.mqtt_clients:
            self.mqtt_clients[broker_host].publish("vanetza/in/denm", json.dumps(denm_payload), qos=0)
            print(f" [SimulatorCore] DENM Injetado via '{vehicle_name}' no canal vanetza/in/denm")

    def step(self) -> None:
        """Executa a sincronização e avanço de 1 frame de simulação (200ms)."""
        self.simulation_time += self.delta_time
        
        # 1. Movimentação física local de cada veículo
        for vehicle in self.vehicles.values():
            # Executa a física interna e regras autónomas baseadas em CAMs vizinhos
            action = vehicle.step(self.delta_time, proximity_threshold=45.0)
            
            # 2. Resposta imediata se o veículo entrou autonomamente em BURST por travagem de risco
            if action == "COLLISION_BURST_ACTIVE":
                if vehicle.should_publish_collision_denm():
                    denm_payload = vehicle.generate_collision_denm_payload()
                    self.publish_denm(vehicle.name, denm_payload)
                    vehicle.mark_collision_denm_published()

        # 3. Processamento do teu Motor de Eventos Extra (Cenários/Acidentes forçados)
        newly_triggered = self.event_engine.update(self.vehicles, self.simulation_time)
        
        for event in newly_triggered:
            # Para cada veículo afetado por este gatilho global, aplica as consequências
            for v_name in event.affected_vehicles:
                if v_name in self.vehicles:
                    tgt_vehicle = self.vehicles[v_name]
                    
                    if event.event_type == "ACCIDENT":
                        # Imobiliza o carro se o evento for um acidente forçado
                        tgt_vehicle.immobilize_for_accident()
                        # Se for o gerador primário do perigo, força a geração do payload
                        if v_name == event.affected_vehicles[0]:
                            # Caso o teu veículo tenha um método para acidentes:
                            payload = tgt_vehicle.generate_accident_denm_payload()
                            self.publish_denm(v_name, payload)
                            print(f" [SimulatorCore] Veículo {v_name} imobilizado por colisão forçada.")

        # 4. Transmissão periódica de infraestrutura de CAMs (5 Hz)
        self.publish_cam_for_all_vehicles()
        self.tick_count += 1

    def run(self, duration_seconds: float) -> None:
        """Loop executor em tempo real com barreira de proteção térmica."""
        self.load_scenario()
        self.setup_mqtt()
        
        max_ticks = int(duration_seconds / self.delta_time)
        print(f"\n [SimulatorCore] Inicialização concluída. A rodar {duration_seconds}s ({max_ticks} ticks)...")
        
        try:
            while self.tick_count < max_ticks:
                start_tick_wall = time.time()
                
                self.step()
                
                # Garante que o relógio físico acompanha os 200ms lógicos
                elapsed = time.time() - start_tick_wall
                sleep_time = max(0.0, self.delta_time - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n [SimulatorCore] Simulação abortada manualmente pelo utilizador.")
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        print("\n [SimulatorCore] A fechar sockets e a desligar clientes MQTT...")
        for host, client in self.mqtt_clients.items():
            client.loop_stop()
            client.disconnect()
        print(" [SimulatorCore] Recursos limpos. Simulação terminada com sucesso.")
