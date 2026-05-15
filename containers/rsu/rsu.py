"""RSU Agent - Monitor de Tráfego Descentralizado."""

import json
import time
import sys
import signal

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("ERROR: paho-mqtt not installed. Run: pip install paho-mqtt")
    sys.exit(1)
import os

RUNNING = True


def signal_handler(_signum, _frame):
    global RUNNING
    RUNNING = False
    print("\n[RSU] Shutting down...")


class RSUAgent:
    """RSU Agent - Monitor de tráfego, observa CAMs e DENMs."""

    def __init__(self):
        self.station_id = 1
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Dados de tráfego conhecidos
        self.known_vehicles = {}
        self.alerts = []
        # connect to broker if needed via VehicleSim not used here; use env for debugging
        self.broker = os.environ.get("MQTT_BROKER", "localhost")

    def on_connect(self, client, userdata, flags, rc):
        """Callback: quando conectado ao MQTT."""
        print(f"[RSU] Conectado ao MQTT broker (rc={rc})")
        self.mqtt_client.subscribe("vanetza/out/cam")
        self.mqtt_client.subscribe("vanetza/out/denm")

    def on_message(self, client, userdata, msg):
        """Callback: quando recebe mensagem MQTT."""
        try:
            payload = json.loads(msg.payload.decode())

            if "cam" in msg.topic:
                station_id = payload.get("stationId")
                self.known_vehicles[station_id] = {
                    "timestamp": time.time(),
                    "payload": payload,
                }

                # Extrair posição
                try:
                    basic = payload["camParameters"]["basicContainer"]["referencePosition"]
                    lat = basic.get("latitude")
                    lon = basic.get("longitude")
                    print(
                        f"[RSU] CAM de Veículo {station_id}: "
                        f"lat={lat:.6f}, lon={lon:.6f}"
                    )
                except Exception:
                    print(f"[RSU] CAM de Veículo {station_id} (posição indisponível)")

            elif "denm" in msg.topic:
                origin = payload.get("management", {}).get("actionId", {}).get("originatingStationId")
                event_pos = payload.get("management", {}).get("eventPosition", {})
                
                alert_msg = (
                    f"[RSU] ALERTA DETECTADO!\n"
                    f"       Origem: Veículo {origin}\n"
                    f"       Posição: lat={event_pos.get('latitude'):.6f}, "
                    f"lon={event_pos.get('longitude'):.6f}\n"
                    f"       Timestamp: {time.strftime('%H:%M:%S')}"
                )
                print("\n" + alert_msg + "\n")
                
                self.alerts.append({
                    "timestamp": time.time(),
                    "origin": origin,
                    "position": event_pos,
                    "payload": payload,
                })

        except Exception as e:
            print(f"[RSU] Erro ao processar mensagem: {e}")

    def run(self):
        """Inicia o agente RSU."""
        print("\n" + "=" * 60)
        print(" RSU AGENTE - MONITOR DE TRÁFEGO")
        print("=" * 60)
        print(f" Station ID: {self.station_id}")
        print(f" Modo: Monitor/Observer")
        print(f" Broker: localhost:1883")
        print("=" * 60 + "\n")

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        try:
            # Conectar ao MQTT local (Vanetza)
            self.mqtt_client.connect("localhost", 1883, keepalive=60)
            print("[RSU] Conectado ao broker MQTT\n")
        except Exception as e:
            print(f"[RSU] Erro ao conectar MQTT: {e}")
            sys.exit(1)

        # Loop MQTT (bloqueia até SIGINT)
        try:
            self.mqtt_client.loop_forever()
        except KeyboardInterrupt:
            pass
        finally:
            print(f"\n[RSU] Total de alertas registados: {len(self.alerts)}")
            print("[RSU] Encerrando...")
            self.mqtt_client.disconnect()


if __name__ == "__main__":
    rsu = RSUAgent()
    rsu.run()
