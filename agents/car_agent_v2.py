import json
import time
import threading
import requests
from typing import Dict, Any, List
import paho.mqtt.client as mqtt

# Importações de domínio e configurações
from domain.vehicle import Vehicle
from domain.eventTrigger import EventTriggerEngine
from scenarios.registry import GLOBAL_SETTINGS

OSRM_SERVER = "http://router.project-osrm.org"

class DecentralizedCarAgent:
    def __init__(self, vehicle_name: str, my_config: Dict[str, Any], scenario_config: Dict[str, Any]):
        self.name = vehicle_name
        self.broker = my_config["broker"]
        self.port = 1883
        
        # 1. Instanciar o SEU próprio veículo passando o dicionário completo (Suporta o novo __init__)
        self.my_vehicle = Vehicle(config=my_config)
        
        # Procurar rota no OSRM de forma autónoma
        self.route = self._fetch_osrm_route(my_config["startPoint"], my_config["endPoint"])
        self.my_vehicle.set_route(self.route)
        
        # 2. Cache local de clones de vizinhos (atualizada via CAMs recebidos por MQTT)
        self.neighbor_vehicles: Dict[str, Vehicle] = {}
        
        # 3. Motor de Eventos Local
        self.event_engine = EventTriggerEngine(scenario_config)
        
        self.mqtt_client = mqtt.Client(client_id=f"agent_{vehicle_name}")
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
        
        if msg.topic == "vanetza/out/denm":
            print(f"[{self.name}] Alerta DENM recebido no ar!")
            self.my_vehicle.denm_registry.register_denm_json(payload_str)
            
        elif msg.topic == "vanetza/out/cam":
            try:
                data = json.loads(payload_str)
                station_id = data["header"]["stationId"]
                
                if station_id == self.my_vehicle.station_id:
                    return  # Ignora mensagens refletidas da própria OBU
                
                # Extrai dados geográficos do Basic Container
                cam_fields = data["cam"]["camParameters"]["basicContainer"]
                v_lat = cam_fields["referencePosition"]["latitude"]
                v_lon = cam_fields["referencePosition"]["longitude"]
                
                # Proteção defensiva para redes Vanetza que devolvem dados inteiros ETSI brutos (escala 10^7)
                if v_lat > 90 or v_lat < -90:
                    v_lat /= 10000000.0
                if v_lon > 180 or v_lon < -180:
                    v_lon /= 10000000.0

                # Extrai cinemática nativa do High Frequency Container
                high_freq = data.get("cam", {}).get("camParameters", {}).get("highFrequencyContainer", {}).get("basicVehicleContainerHighFrequency", {})
                raw_speed = high_freq.get("speed", {}).get("speedValue", 1000)      # Padrão ETSI em cm/s
                raw_heading = high_freq.get("heading", {}).get("headingValue", 0)   # Padrão ETSI em 0.1 graus
                
                neighbor_name = f"station_{station_id}"
                if neighbor_name not in self.neighbor_vehicles:
                    # Instancia clone básico usando os novos argumentos explícitos de vehicle.py
                    self.neighbor_vehicles[neighbor_name] = Vehicle(
                        name=neighbor_name,
                        station_id=station_id,
                        start_point=[v_lat, v_lon],
                        end_point=[v_lat, v_lon],
                        base_speed=0.0
                    )
                
                # 1. Atualiza atributos do clone local direto (Lido pelo Motor de Eventos Local)
                neighbor_vehicle = self.neighbor_vehicles[neighbor_name]
                neighbor_vehicle.current_lat = v_lat
                neighbor_vehicle.current_lon = v_lon
                neighbor_vehicle.speed_mps = raw_speed * 0.01
                neighbor_vehicle.heading = raw_heading * 0.1
                
                # 2. Injeta no buffer de perceção C-ITS do veículo real (Lido pela Tomada de Decisão de travagem)
                self.my_vehicle.update_neighbor_state(station_id, v_lat, v_lon, raw_speed, raw_heading)
                
            except Exception as e:
                pass

    def physics_loop(self):
        """Loop autónomo de 5Hz em tempo lógico sincronizado por hardware."""
        proximity_threshold = GLOBAL_SETTINGS.get("proximity_threshold_meters", 45.0)

        while True:
            start_wall = time.time()
            
            # Sincronização Absoluta Contra Drift: Atualiza sim_time diretamente com o relógio do sistema host
            self.sim_time = time.time()
            
            # 1. Executa o ciclo unificado (Perceção, Decisão e Cinemática)
            self.my_vehicle.step(self.delta_time, proximity_threshold, self.sim_time)
            
            # 2. Publicar o seu CAM atualizado para o Vanetza codificar
            cam_payload = {
                "latitude": self.my_vehicle.current_lat,
                "longitude": self.my_vehicle.current_lon,
                "heading": getattr(self.my_vehicle, 'heading', 0.0),
                "speed": getattr(self.my_vehicle, 'speed_mps', 0.0),
                "station_id": self.my_vehicle.station_id
            }
            self.mqtt_client.publish("vanetza/in/cam", json.dumps(cam_payload))
            
            # 3. Montar perspetiva global para o motor avaliar colisões de forma descentralizada
            all_visible_world = {self.name: self.my_vehicle}
            for neighbor in self.neighbor_vehicles.values():
                all_visible_world[neighbor.name] = neighbor
            
            # Executa avaliação local de regras de risco
            new_triggers = self.event_engine.check_and_update(all_visible_world)
            for trigger in new_triggers:
                if self.name in trigger.affected_vehicles:
                    print(f"[{self.name}] Detetei autonomamente o evento: {trigger.event_type}!")
                    if trigger.event_type == "COLLISION_RISK":
                        self.my_vehicle.start_collision_burst(self.sim_time)

            # Controlo de rajada de DENM (Burst) usando o tempo lógico
            if self.my_vehicle.should_publish_collision_denm(self.sim_time):
                denm_in = {
                    "causeCode": 97, 
                    "subCauseCode": 0,
                    "latitude": self.my_vehicle.current_lat, 
                    "longitude": self.my_vehicle.current_lon
                }
                self.mqtt_client.publish("vanetza/in/denm", json.dumps(denm_in))
                self.my_vehicle.mark_collision_denm_published(self.sim_time)
            
            # Garante cadência estrita de 200ms descontando o tempo de processamento
            elapsed = time.time() - start_wall
            time.sleep(max(0.0, self.delta_time - elapsed))

    def run(self):
        self.setup_mqtt()
        # Inicializa a física em segundo plano
        threading.Thread(target=self.physics_loop, daemon=True).start()
        # Mantém a escuta de pacotes de rádio MQTT ativa por tempo indefinido
        self.mqtt_client.loop_forever()
