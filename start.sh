#!/bin/bash

if [ -d "venv" ]; then
    source venv/bin/activate
elif [ -d ".venv" ]; then
    source .venv/bin/activate
else
    echo "Nenhum ambiente virtual encontrado."
    exit 1
fi

read -r -p "Which of the simulations you wanna test? [collisionRisk : 1 / Intersection : 2] " RUN_SIMULATIONV2X

case "$RUN_SIMULATIONV2X" in
    1)
        #export SIM_SCENARIO="1"
        SIM_SCENARIO="1"
        mosquitto_sub -h 192.168.98.20 -t 'vanetza/out/cam' -v > obu1_cams.log &
        mosquitto_sub -h 192.168.98.21 -t 'vanetza/out/cam' -v > obu2_cams.log &
        mosquitto_sub -h 192.168.98.20 -t 'vanetza/out/denm' -v > obu1_denms.log &
        ;;
    
    2)
        #export SIM_SCENARIO="2"
        SIM_SCENARIO="2"
        mosquitto_sub -h 192.168.98.10 -t 'vanetza/out/cam' -v > rsu_cams.log &
        mosquitto_sub -h 192.168.98.10 -t 'vanetza/out/denm' -v > rsu_denms.log &
        mosquitto_sub -h 192.168.98.20 -t 'vanetza/out/cam' -v > obu1_cams.log &
        mosquitto_sub -h 192.168.98.21 -t 'vanetza/out/cam' -v > obu2_cams.log &
        mosquitto_sub -h 192.168.98.22 -t 'vanetza/out/cam' -v > obu3_cams.log &
        mosquitto_sub -h 192.168.98.23 -t 'vanetza/out/cam' -v > obu4_cams.log &
        ;;

    *)
        echo "Invalid option. Exiting."
        exit 1
        ;;
esac

echo "Starting system..."

python3 simulador.py --scenario "$SIM_SCENARIO" &> simulator.log &

uvicorn backend:app --host 0.0.0.0 --port 8000 &> backend.log &

echo "--------------------------------------------------"
echo Web App running at http://localhost:8000
echo "--------------------------------------------------"
