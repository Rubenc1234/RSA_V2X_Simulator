import asyncio, json, os
import paho.mqtt.client as mqtt
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from datetime import datetime

from scenarios.registry import get_scenario_config

app = FastAPI()
clients: list[WebSocket] = []
SCENARIO_CONFIG = get_scenario_config()
ACTIVE_BROKERS = SCENARIO_CONFIG.get("brokers", [])


def build_station_name_map(brokers: list[dict]) -> dict[int, str]:
    """Build a stationId -> broker name map from the active scenario config."""
    mapping: dict[int, str] = {}
    for index, broker in enumerate(brokers, start=1):
        station_id = broker.get("stationId", index)
        mapping[int(station_id)] = broker.get("name", f"station_{station_id}")
    return mapping


STATION_ID_MAP = build_station_name_map(ACTIVE_BROKERS)

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

def get_obu_name_from_station_id(station_id: int) -> str:
    """Map Vanetza stationId to OBU name."""
    return STATION_ID_MAP.get(station_id, f"unknown_{station_id}")


def handle_cam_message(obu_name: str, payload: dict) -> None:
    """Extract and log CAM details from a Vanetza output payload."""
    cam = payload.get("fields", {}).get("cam", {})
    pos = cam.get("camParameters", {}).get("basicContainer", {}).get("referencePosition", {})
    hfc = cam.get("camParameters", {}).get("highFrequencyContainer", {}).get("basicVehicleContainerHighFrequency", {})

    lat = pos.get("latitude")
    lon = pos.get("longitude")
    speed = hfc.get("speed", {}).get("speedValue")
    heading = hfc.get("heading", {}).get("headingValue")

    # Log valid CAM messages (skip placeholder/invalid coords)
    if lat is not None and lon is not None and lat != 40.0 and lon != -8.0:
        station_id = payload.get("fields", {}).get("header", {}).get("stationId")
        log_cam_message(obu_name, lat, lon, speed, heading, station_id, payload)


def handle_denm_message(obu_name: str, payload: dict) -> None:
    """Extract and log DENM details from a Vanetza output payload."""
    denm = payload.get("fields", {}).get("denm", {})
    mgmt = denm.get("management", {})
    pos = mgmt.get("eventPosition", {})
    sit = denm.get("situation", {})
    event_type = sit.get("eventType", {})
    cc_and_scc = event_type.get("ccAndScc", {})

    lat = pos.get("latitude")
    lon = pos.get("longitude")
    validity = mgmt.get("validityDuration")

    if event_type.get("causeCode") is None and event_type.get("subCauseCode") is None and cc_and_scc:
        cause_desc = ", ".join(f"{key}={value}" for key, value in cc_and_scc.items())
    else:
        cause_desc = (
            f"causeCode={event_type.get('causeCode')} "
            f"subCauseCode={event_type.get('subCauseCode')}"
        )

    print(
        f"[DENM] {obu_name} → lat={lat} lon={lon} "
        f"{cause_desc} validity={validity}s"
    )

def make_mqtt_client(broker_host: str, broker_name: str):
    client = mqtt.Client(client_id=f"webapp-{broker_name}")

    def on_message(_c, _u, msg):
        try:
            payload = json.loads(msg.payload)
            header = payload.get("fields", {}).get("header", {})
            station_id = header.get("stationId")
            obu_name = get_obu_name_from_station_id(station_id) if station_id is not None else "unknown"

            fields = payload.get("fields", {})
            if "cam" in fields:
                handle_cam_message(obu_name, payload)
            elif "denm" in fields:
                handle_denm_message(obu_name, payload)

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
    if data.get("topic") == "vanetza/out/denm":
        obu = data.get("obu", "unknown")
        print(f"[WS] forwarding DENM for {obu} to {len(clients)} frontend client(s)")

    dead = []
    for ws in clients:
        try:
            await ws.send_json(data)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.remove(ws)

@app.on_event("startup")
async def startup():
    global loop
    loop = asyncio.get_event_loop()
    print(f"[backend] starting with scenario={SCENARIO_CONFIG.get('name')} brokers={len(ACTIVE_BROKERS)}")
    for broker in ACTIVE_BROKERS:
        make_mqtt_client(broker["host"], broker["name"])


@app.get("/config")
def config():
    return {
        "name": SCENARIO_CONFIG.get("name"),
        "label": SCENARIO_CONFIG.get("label"),
        "mapCenter": SCENARIO_CONFIG.get("mapCenter"),
        "mapZoom": SCENARIO_CONFIG.get("mapZoom", 17),
        "brokers": ACTIVE_BROKERS,
        "stations": STATION_ID_MAP,
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
