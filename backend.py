import asyncio, json
import paho.mqtt.client as mqtt
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

app = FastAPI()
clients: list[WebSocket] = []

def make_mqtt_client(broker_host: str, obu_name: str):
    client = mqtt.Client(client_id=f"webapp-{obu_name}")

    def on_message(_c, _u, msg):
        try:
            payload = json.loads(msg.payload)
            envelope = {"obu": obu_name, "topic": msg.topic, "payload": payload}
            # Bridge to WebSocket clients (thread-safe via asyncio)
            asyncio.run_coroutine_threadsafe(broadcast(envelope), loop)
        except Exception:
            pass

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
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.remove(ws)

@app.on_event("startup")
async def startup():
    global loop
    loop = asyncio.get_event_loop()
    make_mqtt_client("192.168.98.20", "obu1")
    make_mqtt_client("192.168.98.21", "obu2")

@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.append(websocket)
    try:
        while True:
            await websocket.receive_text()  # keep alive
    except WebSocketDisconnect:
        clients.remove(websocket)

@app.get("/")
def index():
    return FileResponse("index.html")