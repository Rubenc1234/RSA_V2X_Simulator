#!/bin/bash

if [ -d "venv" ]; then
    source venv/bin/activate
elif [ -d ".venv" ]; then
    source .venv/bin/activate
else
    echo "Nenhum ambiente virtual encontrado."
    exit 1
fi

NONINTERACTIVE="${NONINTERACTIVE:-0}"
SIM_SCENARIO="${SIM_SCENARIO:-1}"
START_VANETZA="${START_VANETZA:-n}"

echo "Active scenario: ${SIM_SCENARIO}"

if [[ "$NONINTERACTIVE" != "1" ]]; then
    read -r -p "Which simulation to test? [1=collisionRisk / 2=intersection] [${SIM_SCENARIO}] " RUN_SIMULATIONV2X
    case "$RUN_SIMULATIONV2X" in
        1) SIM_SCENARIO="1" ;;
        2) SIM_SCENARIO="2" ;;
        "") : ;;
        *) echo "Invalid option. Exiting."; exit 1 ;;
    esac

    read -r -p "Start Vanetza Docker containers? [y/N] " START_VANETZA
fi

if [[ "$START_VANETZA" =~ ^[Yy]$ ]]; then
    if [ -f "vanetza-nap/docker compose.yml" ]; then
        echo "Starting Vanetza containers (docker compose)..."
        (cd vanetza-nap && docker compose up -d rsu obu1 obu2 obu3 obu4)
        echo "Vanetza started. Waiting 5s for brokers to be ready..."
        sleep 5
    else
        echo "vanetza-nap/docker compose.yml not found. Skipping Vanetza start."
    fi
fi

echo "Starting mosquitto_sub logs for selected scenario..."
if [ "$SIM_SCENARIO" = "1" ]; then
    mosquitto_sub -h 192.168.98.20 -t 'vanetza/out/cam' -v > obu1_cams.log &
    mosquitto_sub -h 192.168.98.21 -t 'vanetza/out/cam' -v > obu2_cams.log &
    mosquitto_sub -h 192.168.98.20 -t 'vanetza/out/denm' -v > obu1_denms.log &
    mosquitto_sub -h 192.168.98.21 -t 'vanetza/out/denm' -v > obu2_denms.log &
else
    mosquitto_sub -h 192.168.98.10 -t 'vanetza/out/cam' -v > rsu_cams.log &
    mosquitto_sub -h 192.168.98.10 -t 'vanetza/out/denm' -v > rsu_denms.log &
    mosquitto_sub -h 192.168.98.20 -t 'vanetza/out/cam' -v > obu1_cams.log &
    mosquitto_sub -h 192.168.98.21 -t 'vanetza/out/cam' -v > obu2_cams.log &
    mosquitto_sub -h 192.168.98.22 -t 'vanetza/out/cam' -v > obu3_cams.log &
    mosquitto_sub -h 192.168.98.23 -t 'vanetza/out/cam' -v > obu4_cams.log &
fi

echo "Starting backend (uvicorn) locally..."
nohup uvicorn backend:app --host 0.0.0.0 --port 8000 > backend.log 2>&1 &

echo "--------------------------------------------------"
echo "Web App running at http://localhost:8000 (backend.log)"
echo "Backend PID: $(cat backend.pid)"
echo "MQTT logs are in *_cams.log and *_denms.log" 
echo "--------------------------------------------------"
