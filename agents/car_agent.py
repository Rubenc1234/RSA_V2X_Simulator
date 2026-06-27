"""Shared CarAgent used by OBU containers.

This centralises the agent logic so each OBU container can be a thin
bootstrap that loads a profile and runs the same implementation.
"""
from __future__ import annotations

import json
import time
import threading
from types import SimpleNamespace
from typing import Optional

import paho.mqtt.client as mqtt
import math

import simulator_core as core
from simulator_core import (
    VehicleSim,
    TICK_SECONDS,
    DENM_CAUSE_ACCIDENT,
    DENM_CAUSE_COLLISION_RISK,
    DENM_CAUSE_EMERGENCY,
    CAM_TOPIC_IN,
    DENM_TOPIC_IN,
    DENM_TOPIC_OUT,
)
from agents.vehicle_roles import VehicleRole, CollisionBehavior, AccidentBehavior, CorridorBehavior

import agents.cam_denm_codec as cam_denm_codec
import agents.incident_logic as incident_logic
import agents.collision_logic as collision_logic
import agents.corridor_logic as corridor_logic
from agents.avoidance_state import clear_if_expired

from agents.config import INCIDENT_VALIDITY_DURATION_S, INCIDENT_RESET_SECONDS


class CarAgent:
    def __init__(self, profile: dict):
        self.profile = profile
        self.broker = profile.get("broker", "127.0.0.1")
        self.port = int(profile.get("port", 1883))
        self.name = profile.get("name", "UnknownVehicle")

        # Configuração do papel do veículo
        role_str = profile.get("role", "normal")
        try:
            self.role = VehicleRole(role_str)
        except ValueError:
            self.role = VehicleRole.NORMAL

        # Inicialização dos comportamentos (Dataclasses) conforme o contrato B.3
        self.collision = CollisionBehavior()
        collision_data = profile.get("collision", {})
        if isinstance(collision_data, dict):
            if "enabled" in collision_data: self.collision.enabled = collision_data["enabled"]
            if "emitCollisionRiskDenm" in collision_data: self.collision.emit_collision_risk_denm = collision_data["emitCollisionRiskDenm"]
            if "denmStopDistanceM" in collision_data: self.collision.denm_stop_distance_m = float(collision_data["denmStopDistanceM"])
            if "denmSlowdownDistanceM" in collision_data: self.collision.denm_slowdown_distance_m = float(collision_data["denmSlowdownDistanceM"])
            if "denmIgnoreDistanceM" in collision_data: self.collision.denm_ignore_distance_m = float(collision_data["denmIgnoreDistanceM"])

        self.accident = AccidentBehavior()
        accident_data = profile.get("accident", {})
        if isinstance(accident_data, dict):
            if "hardStopOnAccident" in accident_data: self.accident.hard_stop_on_accident = accident_data["hardStopOnAccident"]
            if "rerouteOnAccident" in accident_data: self.accident.reroute_on_accident = accident_data["rerouteOnAccident"]
            if "reroutePreferAlternative" in accident_data: self.accident.reroute_prefer_alternative = accident_data["reroutePreferAlternative"]
            if "accidentStopDistanceM" in accident_data: self.accident.accident_stop_distance_m = float(accident_data["accidentStopDistanceM"])
            if "accidentIgnoreDistanceM" in accident_data: self.accident.accident_ignore_distance_m = float(accident_data["accidentIgnoreDistanceM"])
            if "incidentOnArrival" in accident_data: self.accident.incident_on_arrival = accident_data["incidentOnArrival"]
            if "incidentAfterSeconds" in accident_data:
                val = accident_data["incidentAfterSeconds"]
                self.accident.incident_after_seconds = float(val) if val is not None else None
            if "incidentCauseCode" in accident_data: self.accident.incident_cause_code = int(accident_data["incidentCauseCode"])
            if "incidentSubCauseCode" in accident_data: self.accident.incident_sub_cause_code = int(accident_data["incidentSubCauseCode"])
            if "incidentValidityDuration" in accident_data: self.accident.incident_validity_duration = int(accident_data["incidentValidityDuration"])
            if "incidentResetSeconds" in accident_data: self.accident.incident_reset_seconds = float(accident_data["incidentResetSeconds"])

        # CorridorBehavior inicializado estritamente via Factory do vehicle_roles.py
        self.corridor = CorridorBehavior.from_profile(profile, self.role)

        # Mapeamento de booleanos internos para manter compatibilidade com o simulador local se necessário
        self.is_emergency = (self.role == VehicleRole.EMERGENCY)
        self.react_to_emergency_vehicle = (self.role == VehicleRole.EMERGENCY_REACTOR) or (self.corridor is not None and not self.is_emergency)

        # Configuração do Corridor se ativo
        self.lane = None
        self.lane_offset_m = 0.0
        if self.corridor is not None:
            print(f"[{self.name}] Corridor behavior activated (lane={self.corridor.lane})")
            self.lane = self.corridor.lane
            self.lane_offset_m = self.corridor.lane_offset_m
        else:
            print(f"[{self.name}] No corridor behavior for role {self.role}")

        # ── Construção do veículo simulado com a assinatura REAL do VehicleSim ──
        self.vehicle = VehicleSim(
            name=self.name,
            station_id=int(profile.get("stationId", 0)),
            broker_host=self.broker,
            start_point=tuple(profile.get("startPoint")),
            end_point=tuple(profile.get("endPoint")),
            base_speed_mps=float(profile.get("baseSpeedMps", 5.0)),
            loop_route=bool(profile.get("loopRoute", True)),
        )

        # Estado interno de incidentes e peers detectados
        self.is_stopped_by_accident_denm = False
        self.is_stopped_by_collision_risk = False
        self.accident_detected = False
        self.accident_location = None
        self.has_applied_emergency_lane_change = False

        self.peer_vehicles: dict[int, SimpleNamespace] = {}
        self.peer_lock = threading.Lock()
        self.peer_clients = []

        # ── Estado de runtime usado pelos módulos de lógica (collision/incident/corridor) ──
        self.avoidance_active = False
        self.yield_mode = "stop"
        self.yield_vehicle_name = ""
        self.denm_hold_until = 0.0
        self.last_published_time = 0.0
        self.last_published_event_pos = None
        self.incident_triggered = False
        self.incident_triggered_at = 0.0
        self.yield_until = 0.0
        self.current_lane = self.lane
        self.base_route = list(self.vehicle.route)
        self.base_segment_idx = 0

        # Configuração do Cliente MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Timestamp de início para controlo de incidentes temporizados
        self.start_time = time.time()
        self._emergency_last_published = 0.0

    def attach_peer_listener(self, broker_host: str, client_suffix: str):
        client = mqtt.Client(client_id=client_suffix)
        client.on_message = self.on_message
        try:
            client.connect(broker_host, self.port, keepalive=60)
        except Exception as e:
            print(f"[{self.name}] Erro ao ligar ao broker peer {broker_host}:{self.port}: {e}")
            return
        client.subscribe("vanetza/out/cam")
        client.subscribe(DENM_TOPIC_OUT)
        client.loop_start()
        self.peer_clients.append(client)
        print(f"[{self.name}] Subscribed to peer broker {broker_host}")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"[{self.name}] Conectado com sucesso. Subscrevendo tópicos...")
            client.subscribe("vanetza/out/cam")
            client.subscribe(DENM_TOPIC_OUT)
        else:
            print(f"[{self.name}] Falha na ligação. Código: {rc}")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception as e:
            print(f"[{self.name}] Erro ao decodificar JSON no tópico {msg.topic}: {e}")
            return

        # 1. Receção de CAM (Visão de outros veículos)
        if msg.topic == "vanetza/out/cam":
            station_id = payload.get("fields", {}).get("header", {}).get("stationId")
            if station_id == self.vehicle.station_id:
                return  # Ignora mensagens do próprio veículo

            # Delega puramente a extração e mapeamento para o cam_denm_codec
            peer_view = cam_denm_codec.build_peer_vehicle_view(payload)
            if peer_view:
                with self.peer_lock:
                    self.peer_vehicles[station_id] = peer_view

        # 2. Receção de DENM (Alertas e Eventos da Infraestrutura/Outros Veículos)
        elif msg.topic == DENM_TOPIC_OUT:
            station_id = payload.get("fields", {}).get("header", {}).get("stationId")
            if station_id == self.vehicle.station_id:
                return  # Ignora os seus próprios alertas saídos do Broker

            fields = payload.get("fields", {})
            denm_part = fields.get("denm", {})
            situation = denm_part.get("situation", {})
            event_type = situation.get("eventType", {})
            cause_code = event_type.get("causeCode")
            cc_and_scc = event_type.get("ccAndScc", {})

            if cause_code is None:
                if "accident2" in cc_and_scc:
                    cause_code = DENM_CAUSE_ACCIDENT
                elif "collisionRisk97" in cc_and_scc:
                    cause_code = DENM_CAUSE_COLLISION_RISK
                elif any("emergencyVehicleApproaching" in k for k in cc_and_scc):
                    cause_code = DENM_CAUSE_EMERGENCY

            management = denm_part.get("management", {})
            event_pos_container = management.get("eventPosition", {})
            event_lat = event_pos_container.get("latitude")
            event_lon = event_pos_container.get("longitude")

            if event_lat is None or event_lon is None:
                return

            with self.peer_lock:
                peer_vehicle = self.peer_vehicles.get(station_id)

            # Despacho estruturado consoante o causeCode conforme planeado
            if cause_code == DENM_CAUSE_COLLISION_RISK:
                collision_logic.apply_collision_risk_reaction(self, peer_vehicle, event_lat, event_lon)
            
            elif cause_code == DENM_CAUSE_ACCIDENT:
                collision_logic.apply_accident_reaction(self, peer_vehicle, event_lat, event_lon)
            
            elif cause_code == DENM_CAUSE_EMERGENCY:
                if self.react_to_emergency_vehicle:
                    corridor_logic.react_to_emergency_vehicle(self, (event_lat, event_lon))
            
            else:
                # Fallback de cedência suave se aplicável
                collision_logic.apply_soft_yield_reaction(self, peer_vehicle, event_lat, event_lon)

    def publish_cam_loop(self):
        print(f"[{self.name}] Loop CAM/Orquestração iniciado.")
        while True:
            # 1. Avanço físico do veículo e publicação do CAM básico
            self.vehicle.step_and_publish(TICK_SECONDS)
            clear_if_expired(self)

            # 2. Gestão e Orquestração de Incidentes (Acidentes de Percurso)
            if self.incident_triggered and self.incident_triggered_at > 0:
                incident_logic.maybe_reset_after_accident(self)

            if self.accident is not None:
                is_finished = self.vehicle.segment_idx >= len(self.vehicle.route) - 1

                if self.accident.incident_on_arrival and is_finished and not self.incident_triggered:
                    print(f"[{self.name}] Acidente programado: on_arrival")
                    incident_logic.trigger_accident_denm(self, "arrival")

                if self.accident.incident_after_seconds is not None and not self.incident_triggered:
                    threshold = float(self.accident.incident_after_seconds)
                    elapsed = time.time() - self.start_time
                    if elapsed >= threshold:
                        print(f"[{self.name}] Acidente programado: after {threshold}s (elapsed {elapsed:.1f}s)")
                        incident_logic.trigger_accident_denm(self, "timer")

            # 3. Gestão e Orquestração do Corredor de Emergência
            if self.is_emergency and self.corridor is not None:
                now = time.time()
                if now - self._emergency_last_published >= self.corridor.emergency_denm_interval_s:
                    corridor_logic.publish_emergency_denm(self)
                    self._emergency_last_published = now

            time.sleep(TICK_SECONDS)

    def collision_detection_loop(self):
        print(f"[{self.name}] Loop de Deteção de Colisão iniciado.")
        while True:
            # Orquestra chamando a iteração única do módulo de colisão
            collision_logic.run_detection_tick(self)
            time.sleep(0.5)

    def run(self):
        print("\n" + "=" * 60)
        print(f" INICIANDO AGENTE: {self.name} (ID: {self.vehicle.station_id})")
        print(f" Papel (Role): {self.role.value.upper()}")
        print(f" Rota: {self.vehicle.start_point} → {self.vehicle.end_point}")
        print(f" Broker: {self.broker}:{self.port}")
        if self.lane is not None:
            print(f" Lane: {self.lane} (offset={self.lane_offset_m:.1f}m)")
        print(f" Deteção de colisão: {'ligada' if self.collision.enabled else 'desligada'}")
        print("=" * 60 + "\n")

        try:
            self.mqtt_client.connect(self.broker, self.port, keepalive=60)
            print(f"[{self.name}] Conectado ao broker MQTT\n")
        except Exception as e:
            print(f"[{self.name}] Erro ao conectar MQTT: {e}")
            return

        # Disparo das threads de orquestração
        threading.Thread(target=self.publish_cam_loop, daemon=True).start()
        threading.Thread(target=self.collision_detection_loop, daemon=True).start()

        try:
            self.mqtt_client.loop_forever()
        except KeyboardInterrupt:
            pass
        finally:
            print(f"\n[{self.name}] Agente terminado.")
