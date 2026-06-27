#!/bin/bash

if [ -d "venv" ]; then
    source venv/bin/activate
elif [ -d ".venv" ]; then
    source .venv/bin/activate
elif [[ "${NONINTERACTIVE:-0}" != "1" ]]; then
    echo "Nenhum ambiente virtual encontrado."
    exit 1
fi

NONINTERACTIVE="${NONINTERACTIVE:-0}"
SIM_SCENARIO="${SIM_SCENARIO:-1}"
START_VANETZA="${START_VANETZA:-n}"

echo "Active scenario: ${SIM_SCENARIO}"

get_scenario_brokers() {
    python - "$SIM_SCENARIO" <<'PY'
import sys
from scenarios.registry import get_scenario_config

scenario = get_scenario_config(sys.argv[1])
for broker in scenario.get("brokers", []):
    print(f"{broker.get('name','')}|{broker.get('host','')}")
PY
}

start_vanetza_containers() {
    local brokers_list
    brokers_list="$(get_scenario_brokers)"
    if [[ -z "$brokers_list" ]]; then
        echo "No brokers found for scenario ${SIM_SCENARIO}. Skipping Vanetza start."
        return
    fi

    local broker_names=()
    while IFS='|' read -r broker_name broker_host; do
        if [[ -n "$broker_name" ]]; then
            broker_names+=("$broker_name")
        fi
    done <<< "$brokers_list"

    if [[ ${#broker_names[@]} -eq 0 ]]; then
        echo "No broker names resolved for scenario ${SIM_SCENARIO}. Skipping Vanetza start."
        return
    fi

    echo "Starting Vanetza containers (${broker_names[*]})..."
    (cd vanetza-nap && docker compose up -d "${broker_names[@]}")
    echo "Vanetza started. Waiting 5s for brokers to be ready..."
    sleep 5
}

start_mqtt_logs() {
    local brokers_list
    brokers_list="$(get_scenario_brokers)"
    if [[ -z "$brokers_list" ]]; then
        echo "No brokers found for scenario ${SIM_SCENARIO}. Skipping MQTT log subscriptions."
        return
    fi

    while IFS='|' read -r broker_name broker_host; do
        if [[ -z "$broker_name" || -z "$broker_host" ]]; then
            continue
        fi

        local safe_name
        safe_name="${broker_name//[^a-zA-Z0-9_-]/_}"
        mosquitto_sub -h "$broker_host" -t 'vanetza/out/cam' -v > "${safe_name}_cams.log" &
        mosquitto_sub -h "$broker_host" -t 'vanetza/out/denm' -v > "${safe_name}_denms.log" &
    done <<< "$brokers_list"
}

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
    if [ -f "vanetza-nap/docker-compose.yml" ]; then
        start_vanetza_containers
    else
        echo "vanetza-nap/docker-compose.yml not found. Skipping Vanetza start."
    fi
fi

echo "Starting mosquitto_sub logs for selected scenario..."
start_mqtt_logs

echo "Starting backend (uvicorn) locally..."
nohup uvicorn backend:app --host 0.0.0.0 --port 8000 > backend.log 2>&1 &

echo "--------------------------------------------------"
echo "Web App running at http://localhost:8000 (backend.log)"
echo "Backend PID: $(cat backend.pid)"
echo "MQTT logs are in *_cams.log and *_denms.log" 
echo "--------------------------------------------------"
