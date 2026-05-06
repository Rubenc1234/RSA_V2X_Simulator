#!/bin/bash

if [ -d "venv" ]; then
    source venv/bin/activate
elif [ -d ".venv" ]; then
    source .venv/bin/activate
else
    echo "Nenhum ambiente virtual encontrado."
    exit 1
fi

echo "Starting system..."

python3 simulador.py &> simulator.log &

mosquitto_sub -h 192.168.98.21 -t 'vanetza/out/cam' -v > obu2_cams.log &
mosquitto_sub -h 192.168.98.20 -t 'vanetza/out/cam' -v > obu1_cams.log &

uvicorn backend:app --host 0.0.0.0 --port 8000 &> backend.log &

echo "--------------------------------------------------"
echo Web App running at http://localhost:8000
echo "--------------------------------------------------"
