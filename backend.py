import asyncio, json, os
import paho.mqtt.client as mqtt
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from datetime import datetime

from scenarios.registry import get_scenario_config

app = FastAPI()
clients: list[WebSocket] = []
SCENARIO_CONFIG = get_scenario_config()

# Setup logging directories
os.makedirs("logs", exist_ok=True)

def log_cam_message(obu_name: str, lat: float, lon: float, speed: float, heading: float, station_id: int, raw_payload: dict):
    """Log CAM message to both file and console."""
    timestamp = datetime.now().isoformat()
    
    # Structured log entry
    log_entry = {
        "timestamp": timestamp,
        "obu": obu_name,
        "stationId": station_id,
        "lat": lat,
        "lon": lon,
        "speed": speed,
        "heading": heading,
    }
    
    # Write to OBU-specific log file
    log_file = f"logs/{obu_name}_cam_log.jsonl"
    with open(log_file, "a") as f:
        f.write(json.dumps(log_entry) + "\n")
    
    # Console output
    print(f"[{timestamp}] {obu_name} → lat={lat:.6f} lon={lon:.6f} speed={speed} m/s hdg={heading}°")

STATION_ID_MAP = {
    1: "rsu",
    2: "obu1",
    3: "obu2",
    4: "obu3",
    5: "obu4"
}

def get_obu_name_from_station_id(station_id: int) -> str:
    """Map Vanetza stationId to OBU name."""
    return STATION_ID_MAP.get(station_id, f"unknown_{station_id}")

def make_mqtt_client(broker_host: str, broker_name: str):
    client = mqtt.Client(client_id=f"webapp-{broker_name}")

    def on_message(_c, _u, msg):
        try:
            payload = json.loads(msg.payload)
            
            # Extract stationId from the decoded message header
            station_id = payload.get("fields", {}).get("header", {}).get("stationId")
            obu_name = get_obu_name_from_station_id(station_id) if station_id else f"station_{station_id}"
            
            # Process CAM messages
            if "cam" in payload.get("fields", {}):
                cam = payload.get("fields", {}).get("cam", {})
                pos = cam.get("camParameters", {}).get("basicContainer", {}).get("referencePosition", {})
                hfc = cam.get("camParameters", {}).get("highFrequencyContainer", {}).get("basicVehicleContainerHighFrequency", {})
                
                lat = pos.get("latitude")
                lon = pos.get("longitude")
                speed = hfc.get("speed", {}).get("speedValue")
                heading = hfc.get("heading", {}).get("headingValue")
                
                # Log valid CAM messages (skip placeholder/invalid coords)
                if lat is not None and lon is not None and lat != 40.0 and lon != -8.0:
                    log_cam_message(obu_name, lat, lon, speed, heading, station_id, payload)
            
            # Process DENM messages
            elif "denm" in payload.get("fields", {}):
                denm = payload.get("fields", {}).get("denm", {})
                mgmt = denm.get("management", {})
                pos = mgmt.get("eventPosition", {})
                sit = denm.get("situation", {})
                event_type = sit.get("eventType", {})
                cc_and_scc = event_type.get("ccAndScc", {})
                
                lat = pos.get("latitude")
                lon = pos.get("longitude")
                cause_code = event_type.get("causeCode")
                sub_cause_code = event_type.get("subCauseCode")
                validity = mgmt.get("validityDuration")

                if cause_code is None and sub_cause_code is None and cc_and_scc:
                    cause_desc = ", ".join(f"{key}={value}" for key, value in cc_and_scc.items())
                else:
                    cause_desc = f"causeCode={cause_code} subCauseCode={sub_cause_code}"
                
                print(
                    f"[DENM] {obu_name} → lat={lat} lon={lon} "
                    f"{cause_desc} validity={validity}s"
                )
            
            envelope = {"obu": obu_name, "topic": msg.topic, "payload": payload}
            # Bridge to WebSocket clients (thread-safe via asyncio)
            asyncio.run_coroutine_threadsafe(broadcast(envelope), loop)
        except Exception as e:
            print(f"Error processing message: {e}")

    client.on_message = on_message
    client.connect(broker_host, 1883, 60)
    client.subscribe("vanetza/out/cam")
    client.subscribe("vanetza/out/denm")
    client.loop_start()
    return client

async def broadcast(data: dict):
    dead = []
    for ws in clients:
        try:
            await ws.send_json(data)
            if data.get("topic") == "vanetza/out/denm":
                obu = data.get("obu", "unknown")
                print(f"[WS] forwarded DENM for {obu} to frontend clients")
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.remove(ws)

@app.on_event("startup")
async def startup():
    global loop
    loop = asyncio.get_event_loop()
    for broker in SCENARIO_CONFIG.get("brokers", []):
        make_mqtt_client(broker["host"], broker["name"])


@app.get("/config")
def config():
    return {
        "name": SCENARIO_CONFIG.get("name"),
        "label": SCENARIO_CONFIG.get("label"),
        "mapCenter": SCENARIO_CONFIG.get("mapCenter"),
        "mapZoom": SCENARIO_CONFIG.get("mapZoom", 17),
        "brokers": SCENARIO_CONFIG.get("brokers", []),
    }

@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.append(websocket)
    print(f"[WS] frontend connected ({len(clients)} active client(s))")
    try:
        while True:
            await websocket.receive_text()  # keep alive
    except WebSocketDisconnect:
        clients.remove(websocket)
        print(f"[WS] frontend disconnected ({len(clients)} active client(s))")

@app.get("/")
def index():
    return FileResponse("index.html")
