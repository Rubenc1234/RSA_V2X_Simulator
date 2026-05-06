#!/bin/bash

# verificar se te, pasta venv ou .venv
# ativar o ambiente virtual
if [ -d "venv" ]; then
    source venv/bin/activate
elif [ -d ".venv" ]; then
    source .venv/bin/activate
else
    echo "Nenhum ambiente virtual encontrado. Por favor, crie um ambiente virtual usando 'python -m venv venv' ou 'python -m venv .venv'."
    exit 1
fi

python3 simulador.py &> simulator.log &

mosquitto_sub -h 192.168.98.21 -t 'vanetza/out/cam' -v > obu2_cams.log &
mosquitto_sub -h 192.168.98.20 -t 'vanetza/out/cam' -v > obu1_cams.log &
