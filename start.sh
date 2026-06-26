#!/bin/bash

# 1. Escolha do Cenário (Agora usa os nomes reais do teu registry.py)
echo "Qual o cenário que pretendes simular?"
echo "1) collision_risk"
echo "2) accident"
read -p "Opção [1 ou 2] (Default: 1): " OPCAO

case "$OPCAO" in
    2) export SIM_SCENARIO="accident" ;;
    *) export SIM_SCENARIO="collision_risk" ;;
esac

echo "🚀 Cenário Ativo: $SIM_SCENARIO"

# 2. Criar a rede do Docker se ela não existir
docker network inspect vanetzalan0 >/dev/null 2>&1 || {
    echo "🌐 Criando rede vanetzalan0..."
    docker network create vanetzalan0 --subnet=192.168.98.0/24
}

# 3. Arrancar TODA a simulação (Vanetza + Teus Agentes)
echo "🐳 A iniciar os contentores via Docker Compose..."
docker compose up -d --build

echo "⏳ A aguardar 5 segundos para estabilização dos Brokers..."
sleep 5

# 4. [Opcional] Criar os teus ficheiros de Log via mosquitto_sub para Debug
echo "📝 A iniciar a escuta de logs MQTT para verificação (em background)..."
# Exemplo para monitorizar a OBU1 e OBU2 através dos IPs definidos no teu registry.py
mosquitto_sub -h 192.168.98.20 -t 'vanetza/out/cam' -v > obu1_cams.log &
mosquitto_sub -h 192.168.98.20 -t 'vanetza/out/denm' -v > obu1_denms.log &
mosquitto_sub -h 192.168.98.21 -t 'vanetza/out/cam' -v > obu2_cams.log &
mosquitto_sub -h 192.168.98.21 -t 'vanetza/out/denm' -v > obu2_denms.log &

echo "--------------------------------------------------------"
echo "✅ Simulação em execução!"
echo "👉 Podes ver os logs em tempo real do teu agente com: docker logs -f obu1-agent"
echo "👉 Os logs de rede estão a ser gravados em obu1_cams.log, etc."
echo "--------------------------------------------------------"
