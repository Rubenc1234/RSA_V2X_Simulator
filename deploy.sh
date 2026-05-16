#!/bin/bash

# Script de Deploy para Sistema Descentralizado
# Uso: ./deploy.sh [start|stop|logs|build|clean]

set -e

NETWORK_NAME="vanetzalan0"
NETWORK_SUBNET="192.168.98.0/24"

function create_network() {
    echo " Verificando rede Docker..."
    if ! docker network ls | grep -q "$NETWORK_NAME"; then
        echo " Criando rede $NETWORK_NAME..."
        docker network create --subnet="$NETWORK_SUBNET" "$NETWORK_NAME"
        echo " Rede criada com sucesso"
    else
        echo " Rede $NETWORK_NAME já existe"
    fi
}

function cleanup_conflicting_compose_stack() {
    if docker ps -a --format '{{.Names}}' | grep -q '^vanetza-nap_'; then
        echo "  Detectei containers da stack 'vanetza-nap' a usar a mesma rede/IP."
        echo " A remover apenas esses containers para evitar Address already in use..."
        docker rm -f $(docker ps -aq --filter "name=^/vanetza-nap_") >/dev/null 2>&1 || true
    fi
}

function build() {
    echo " Construindo imagens Docker..."
    create_network
    docker-compose build
    echo " Build completo"
}

function start() {
    echo " Iniciando sistema..."
    create_network
    cleanup_conflicting_compose_stack
    
    # Accept optional scenario override passed as first arg to start()
    local requested_scenario="$1"
    if [[ -z "$requested_scenario" ]]; then
        requested_scenario="${SIM_SCENARIO:-1}"
    fi
    case "$requested_scenario" in
        1|2) SIM_SCENARIO="$requested_scenario" ;;
        *)
            echo "Invalid or missing scenario '$requested_scenario'; defaulting to 1"
            SIM_SCENARIO=1
            ;;
    esac

    echo " Iniciando stack principal com docker-compose..."
    docker-compose up -d
    sleep 5

    echo " Iniciando backend local via start.sh (non-interactive, scenario=${SIM_SCENARIO})..."
    NONINTERACTIVE=1 START_VANETZA=n RUN_SIMULATOR=n SIM_SCENARIO="${SIM_SCENARIO}" ./start.sh
    
    echo " Sistema iniciado com sucesso!"
    echo ""
    echo "Logs disponíveis com:"
    echo "  docker-compose logs -f               # Todos os containers Docker"
    echo "  tail -f backend.log                  # Backend local"
}

function stop() {
    echo " Parando sistema..."
    ./stop.sh
    echo " Sistema parado"
}

function logs() {
    echo " Mostrando logs..."
    docker-compose logs -f "$@"
}

function clean() {
    echo " Limpando containers e imagens..."
    docker-compose down --rmi all
    echo " Limpeza completa"
}

function status() {
    echo " Status dos containers:"
    docker-compose ps
}

case "${1:-help}" in
    start)
        # Allow: ./deploy.sh start [1|2]
        start "${2:-}"
        ;;
    stop)
        stop
        ;;
    logs)
        logs "${@:2}"
        ;;
    build)
        build
        ;;
    clean)
        clean
        ;;
    status)
        status
        ;;
    *)
        echo "Descentralizado System Deploy Script"
        echo ""
        echo "Uso: $0 {start|stop|logs|build|clean|status}"
        echo ""
        echo "Comandos:"
        echo "  start      - Inicia o sistema completo"
        echo "  stop       - Para o sistema"
        echo "  logs       - Mostra logs (opcionalmente: 'logs obu1-agent')"
        echo "  build      - Constrói imagens Docker"
        echo "  clean      - Remove containers e imagens"
        echo "  status     - Mostra status dos containers"
        echo ""
        echo "Exemplos:"
        echo "  $0 start              # Iniciar tudo"
        echo "  $0 logs obu1-agent    # Ver logs de OBU1"
        echo "  $0 stop               # Parar tudo"
        exit 1
        ;;
esac
