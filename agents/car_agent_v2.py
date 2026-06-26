import json
import time
import threading
import requests
from typing import Dict, Any, List
import paho.mqtt.client as mqtt

# Importações de domínio e configurações
from domain.vehicle import Vehicle
from domain.eventTrigger import EventTriggerEngine
from scenarios.registry_2 import GLOBAL_SETTINGS
from domain.utils import build_cam_payload

OSRM_SERVER = "http://router.project-osrm.org"

class DecentralizedCarAgent:
    def __init__(self, vehicle_name: str, my_config: Dict[str, Any], scenario_config: Dict[str, Any]):
        self.name = vehicle_name
        self.broker = my_config["broker"]
        self.port = 1883
        
        # 1. Instanciar o SEU próprio veículo passando o dicionário completo (Suporta o novo __init__)
        self.my_vehicle = Vehicle(config=my_config)
        print(f"[{self.name}] Vehicle criado: {self.my_vehicle.current_lat}, {self.my_vehicle.current_lon}")

        # Procurar rota no OSRM de forma autónoma
        self.route = self._fetch_osrm_route(my_config["startPoint"], my_config["endPoint"])
        print(f"[{self.name}] Rota fetched (OSRM): {self.route[:3]}...")
        self.my_vehicle.set_route(self.route)
        print(f"[{self.name}] Rota injetada. Vehicle agora: {self.my_vehicle.current_lat}, {self.my_vehicle.current_lon}")
        
        # 2. Cache local de clones de vizinhos (atualizada via CAMs recebidos por MQTT)
        self.neighbor_vehicles: Dict[str, Vehicle] = {}
        
        # 3. Motor de Eventos Local
        self.event_engine = EventTriggerEngine(scenario_config.get("events", []))
        
        self.mqtt_client = mqtt.Client(client_id=f"agent_{vehicle_name}")
        self.peer_clients: List[mqtt.Client] = []
        self.sim_time = time.time()  # Alinhado inicialmente com o relógio do sistema host
        self.delta_time = 0.2        # 5Hz
        
    def _fetch_osrm_route(self, start: List[float], end: List[float]):
        try:
            url = f"{OSRM_SERVER}/route/v1/driving/{start[1]},{start[0]};{end[1]},{end[0]}?overview=full&geometries=geojson"
            r = requests.get(url, timeout=5).json()
            return [(wp[1], wp[0]) for wp in r["routes"][0]["geometry"]["coordinates"]]
        except Exception as e:
            print(f"[{self.name}] Erro OSRM: {e}")
            return [tuple(start), tuple(end)]
    
    def attach_peer_listener(self, broker_host: str, client_suffix: str):
        # Criar cliente MQTT para o broker vizinho usando a API estável do paho-mqtt
        client = mqtt.Client(client_id=client_suffix)
        client.on_message = self.on_message  # Partilha o mesmo callback de parsing
        
        try:
            client.connect(broker_host, self.port, keepalive=60)
        except Exception as e:
            print(f"[{self.name}] Erro ao ligar ao broker peer {broker_host}:{self.port}: {e}")
            return
            
        # Subscrever o tráfego gerado pelo Vanetza do vizinho
        client.subscribe("vanetza/out/cam")
        client.subscribe("vanetza/out/denm")
        
        # Iniciar a thread em background dedicada a este broker
        client.loop_start()
        self.peer_clients.append(client)
        print(f"[{self.name}] Subscribed com sucesso ao peer broker: {broker_host}")

    def setup_mqtt(self):
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.broker, self.port, 60)
        
    def on_connect(self, client, userdata, flags, rc):
        print(f"[{self.name}] Conectado ao Broker Local. A subscrever outputs do Vanetza...")
        client.subscribe("vanetza/out/cam")
        client.subscribe("vanetza/out/denm")

    def on_message(self, client, userdata, msg):
        payload_str = msg.payload.decode("utf-8")
        print(f"[{self.name}] Mensagem recebida no tópico {msg.topic}: {payload_str[:100]}...")
        
        if msg.topic == "vanetza/out/denm":
            print(f"[{self.name}] Alerta DENM recebido no ar!")
            self.my_vehicle.denm_registry.register_denm_json(payload_str)
            
        elif msg.topic == "vanetza/out/cam":
            try:
                data = json.loads(payload_str)
                
                # Validação segura da estrutura externa (com ou sem o wrapper 'fields')
                fields = data.get("fields", data)
                cam_msg = fields.get("cam", {})
                cam_params = cam_msg.get("camParameters", {})
                
                basic_container = cam_params.get("basicContainer", {})
                
                # Tenta obter o highFrequencyContainer
                hf_block = cam_params.get("highFrequencyContainer", {})
                # Suporta encontrar a chave diretamente ou dentro do sub-container basicVehicleContainerHighFrequency
                hf_container = hf_block.get("basicVehicleContainerHighFrequency", hf_block)
                
                station_id = data.get("stationID") or fields.get("header", {}).get("stationId")
                if not station_id:
                    return

                # 1. Extração e conversão de Latitude e Longitude (Escala 10^7)
                raw_lat = basic_container["referencePosition"]["latitude"]
                raw_lon = basic_container["referencePosition"]["longitude"]
                v_lat = raw_lat / 10000000.0 if abs(raw_lat) > 180 else raw_lat
                v_lon = raw_lon / 10000000.0 if abs(raw_lon) > 180 else raw_lon

                # Ignorar coordenadas default (40.00, -8.00)
                if round(v_lat, 2) == 40.00 and round(v_lon, 2) == -8.00:
                    print(f"[{self.name}] CAM de station_id={station_id} ignorado (Coordenadas Default {v_lat}, {v_lon})")
                    return
                
                # 2. Extração segura de Heading (Tratando se vier como dicionário ou inteiro direto)
                v_heading = 0.0
                if "heading" in hf_container:
                    heading_data = hf_container["heading"]
                    raw_heading = heading_data.get("headingValue", 0) if isinstance(heading_data, dict) else heading_data
                    v_heading = raw_heading * 0.1 if raw_heading <= 3600 else 0.0
                    if v_heading > 360.0:
                        v_heading = 0.0
                        
                # 3. Extração segura de Speed (Tratando se vier como dicionário ou inteiro direto)
                v_speed = 0.0
                if "speed" in hf_container:
                    speed_data = hf_container["speed"]
                    raw_speed = speed_data.get("speedValue", 0) if isinstance(speed_data, dict) else speed_data
                    v_speed = raw_speed * 0.01 if raw_speed < 16383 else 0.0

                # 4. Envio dos dados corrigidos em unidades físicas corretas (m/s e graus)
                self.my_vehicle.update_neighbor_state(
                    station_id, 
                    v_lat, 
                    v_lon, 
                    v_speed, 
                    v_heading
                )

            except KeyError as e:
                print(f"[{self.name}] Erro de chave ao processar CAM: {e}")
            except Exception as e:
                print(f"[{self.name}] Erro genérico no parsing do CAM: {e}")

    
    def physics_loop(self):
        """Loop autónomo de 5Hz em tempo lógico sincronizado por hardware."""
        proximity_threshold = GLOBAL_SETTINGS.get("proximity_threshold_meters", 45.0)
    
        while True:
            start_wall = time.time()
            
            # Sincronização Absoluta Contra Drift
            self.sim_time = time.time()
            
            # 1. Executa o ciclo unificado
            self.my_vehicle.step(self.delta_time, proximity_threshold, self.sim_time)
            
            # 2. PUBLICAR CAM CORRIGIDO
            # Chama build_cam_payload que JÁ retorna a estrutura completa com generationDeltaTime
            cam_payload = build_cam_payload(
                station_id=self.my_vehicle.station_id,
                lat=self.my_vehicle.current_lat,      # Ex: 41.726213
                lon=self.my_vehicle.current_lon,      # Ex: -8.164577
                speed_mps=self.my_vehicle.speed_mps,  # Em m/s
                heading_deg=self.my_vehicle.heading,  # Em graus (0-360)
                station_type=self.my_vehicle.vehicle_type
            )
            print(f"[{self.name}] Publicando CAM: {cam_payload}")
            
            # O payload já contém a estrutura completa:
            # {
            #   "camParameters": {...},
            #   "generationDeltaTime": 12345
            # }
            
            # Enviar diretamente sem nesting extra
            vanetza_in_payload = {
                "cam": cam_payload
            }
            
            self.mqtt_client.publish("vanetza/in/cam", json.dumps(vanetza_in_payload))
            
            # 3. Resto da lógica...
            all_visible_world = {self.name: self.my_vehicle}
            for neighbor in self.neighbor_vehicles.values():
                all_visible_world[neighbor.name] = neighbor
            
            new_triggers = self.event_engine.check_and_update(all_visible_world)
            for trigger in new_triggers:
                if self.name in trigger.affected_vehicles:
                    print(f"[{self.name}] Evento detectado: {trigger.event_type}!")
                    if trigger.event_type == "COLLISION_RISK":
                        self.my_vehicle.start_collision_burst(self.sim_time)
    
            # Controlo de rajada DENM
            if self.my_vehicle.should_publish_collision_denm(self.sim_time):
                denm_in = {
                    "causeCode": 97, 
                    "subCauseCode": 0,
                    "latitude": self.my_vehicle.current_lat, 
                    "longitude": self.my_vehicle.current_lon
                }
                self.mqtt_client.publish("vanetza/in/denm", json.dumps(denm_in))
                self.my_vehicle.mark_collision_denm_published(self.sim_time)
            
            # Garante cadência estrita de 200ms
            elapsed = time.time() - start_wall
            time.sleep(max(0.0, self.delta_time - elapsed))

    def run(self):
        self.setup_mqtt()
        # Inicializa a física em segundo plano
        threading.Thread(target=self.physics_loop, daemon=True).start()
        # Mantém a escuta de pacotes de rádio MQTT ativa por tempo indefinido
        self.mqtt_client.loop_forever()
