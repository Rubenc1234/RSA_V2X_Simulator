# Projeto V2X com Vanetza-NAP

## Visao Geral

Este repositorio implementa um ambiente de simulacao V2X (ETSI C-ITS) com Docker e Vanetza-NAP.
Cada OBU e executada num container, recebe coordenadas GPS emuladas e publica mensagens CAM.

Objetivos do projeto:
- Escalar para multiplas OBUs (um container por veiculo).
- Gerar CAM com base em trajetos GPS emulados.
- Evoluir para leitura em tempo real numa web app com mapa.
- Injetar DENM e modelar reacoes dos veiculos a cada alerta.

## Estado Atual

Disponivel neste momento:
- Infraestrutura Vanetza-NAP em [vanetza-nap](vanetza-nap).
- Orquestracao Docker em [vanetza-nap/docker-compose.yml](vanetza-nap/docker-compose.yml).
- MVP de simulador em [simulador.py](simulador.py), focado em OBUs:
   - 2 OBUs com trajetos GPS simples.
   - Tick de simulacao a 5 Hz.
   - Publicacao de CAM para cada broker MQTT OBU.

## Arquitetura

Fluxo atual (MVP CAM):

Python simulator
-> publica CAM JSON em vanetza/in/cam (por OBU)
-> Vanetza codifica e envia na rede ITS-G5 simulada
-> containers recetores publicam em vanetza/out/cam
-> observacao por mosquitto_sub (e futuramente web app)

## Topicos MQTT

Topicos principais usados nesta fase:
- CAM input: vanetza/in/cam
- CAM output: vanetza/out/cam
- DENM input: vanetza/in/denm
- DENM output: vanetza/out/denm

## Primeiros Passos Recomendados

Foco inicial: OBUs. A RSU pode manter-se como container de suporte no compose.

### Opcao Rapida (Script Automatizado)

Para arrancar tudo de uma vez:

```bash
chmod +x start.sh
./start.sh
# Deixa correr ~30-60 segundos
# Abre browser em http://localhost:8000

# Para parar tudo:
./stop.sh
```

O script `start.sh`:
- Ativa o ambiente virtual Python
- Inicia o simulador em background
- Inicia subscrições MQTT para logs
- Arranca o backend FastAPI com mapa

O script `stop.sh`:
- Para o simulador
- Para as subscrições MQTT
- Para o backend

### Opcao Detalhada (Passo a Passo Manual)

Recomendado para entender cada componente:

1. **Confirmar baseline dos containers**
   - Criar a rede Docker (se necessario).
   - Arrancar os containers Vanetza.
   - Confirmar que ja existem CAM periodicas em pelo menos uma OBU.

2. **Executar o MVP do simulador**
   - Ativar o ambiente Python e garantir dependencia paho-mqtt.
   - Executar o simulador e validar que o loop esta a correr a 5 Hz.
   - Verificar no terminal que as coordenadas das OBUs mudam ao longo do tempo.

3. **Validar o criterio de sucesso**
   - Subscricao MQTT mostra CAM com latitude e longitude dinamicas.
   - O comportamento e observavel para as duas OBUs.

## Setup Rapido

Pre-requisitos:
- Docker e Docker Compose
- Python 3
- Mosquitto clients

Comandos:

1. Instalar dependencias no host
```bash
sudo apt update
sudo apt install -y docker.io docker-compose python3 python3-pip mosquitto-clients
```

2. Criar ambiente Python local

```bash
python3 -m venv venv
source venv/bin/activate
pip install paho-mqtt fastapi requests uvicorn 'uvicorn[standard]'
```

3. Criar rede Docker

```bash
docker network create vanetzalan0 --subnet 192.168.98.0/24
```

4. Levantar containers
```bash
cd vanetza-nap
docker-compose up -d
```

5. **Opcao A - Automatizado (Recomendado)**
   
   De volta à raiz do projeto:
   ```bash
   chmod +x start.sh stop.sh
   chmod +x deploy.sh
   ./deploy build
   ./deploy start <cenário number>
   ```
   
   Abrir http://localhost:8000 no browser.
   Depois de 30-60 segundos,
   
   Para parar tudo:
   ```bash
   ./stop.sh
   docker compose down
   ```

## Componentes Principais

### backend.py
O script [backend.py](backend.py) implementa um agregador MQTT + mapa web:
- Subscreve aos X brokers Vanetza em `vanetza/out/cam` e `vanetza/out/denm`
- Normaliza mensagens e reemite por WebSocket
- Serve mapa Leaflet em tempo real em `http://localhost:8000`
- Mostra X marcadores a moverem de acordo com CAMs recebidos

### simulator_core.py
O motor de simulação matemática e conformidade do ecossistema:
- Define as constantes globais do ambiente de simulação (como a frequência de atualização de `5.0 Hz` e as distâncias de segurança em metros).
- Integra com o servidor OSRM (`http://router.project-osrm.org`) para consumir as rotas geoespaciais e interpolar os passos cinemáticos dos veículos.
- Disponibiliza as funções geográficas fundamentais do sistema, como o cálculo de distâncias pela fórmula de Haversine e a determinação de azimutes (*bearings*).
- Implementa funções auxiliares de construção de *payloads* em conformidade com o formato exigido pela *stack* Vanetza (estruturas de dados para injeção de CAM e DENM).

### car_agent.py
O núcleo descentralizado de decisão reativa de cada veículo:
- Instancia instâncias independentes de `CarAgent` com base no perfil específico atribuído a cada nó.
- Orquestra uma arquitetura multithread paralela através de três ciclos principais (*loops* daemon):
  - **Publish CAM:** Difunde periodicamente o estado cinemático (posição, velocidade e rumo) para a rede.
  - **Collision Detection:** Avalia autonomamente a proximidade e risco em relação aos nós vizinhos.
  - **DENM Listener:** Subscreve e processa os alertas propagados pelos brokers dos *peers*.
- Altera dinamicamente os parâmetros de mobilidade do veículo (como abrandamentos para 40%, paragens completas ou mudanças de faixa) em função dos eventos processados em tempo real.

### registry.py (scenarios/)
O catálogo e repositório central de configurações do simulador:
- Centraliza o dicionário estruturado `SCENARIOS`, que define os mapas base, as coordenadas de georreferenciação iniciais e o nível de zoom.
- Mapeia o ecossistema de rede, associando cada veículo ao seu `stationId` respetivo e ao endereço IP estático do seu *broker* MQTT dedicado.
- Define as propriedades individuais de cada nó (coordenadas de origem/destino, velocidades de cruzeiro e parâmetros de temporização para simulação de sinistros).

### obu1.py (e bootstraps homólogos)
O script de inicialização rápida (*bootstrap*) de cada Unidade de Bordo:
- Atua como um ponto de entrada leve e modular executado no arranque de cada contentor de agente.
- Extrai as variáveis de ambiente necessárias (como o `VEHICLE_NAME`) para carregar o perfil correto a partir do `registry.py`.
- Descobre a topologia do cenário e associa dinamicamente os *brokers* MQTT dos nós vizinhos (*peers*) ao cliente local para viabilizar a escuta cruzada de mensagens ITS antes de iniciar o ciclo principal do agente.

### index.html
A interface gráfica e painel de controlo visual do utilizador (*frontend*):
- Renderiza um mapa interativo bidimensional em ambiente *web* recorrendo à biblioteca JavaScript Leaflet.
- Estabelece uma ligação persistente via WebSockets com o `backend.py` para consumir o fluxo unificado de dados geográficos.
- Desenha e atualiza em tempo real os marcadores dinâmicos dos veículos, os raios geométricos de alcance e a sinalização visual de alertas críticos (como ícones de colisão ou de acidente) respeitando os tempos de validade das mensagens DENM.

## Referencias

- Vanetza-NAP: https://github.com/nap-it/vanetza-nap
- ETSI C-ITS: https://www.etsi.org/
- MQTT: https://mqtt.org/
- Leaflet: https://leafletjs.com/

## Contexto Academico

Projeto da UC Redes e Sistemas Autonomos (RSA), Mestrado em Engenharia de Computadores e Telematica, Universidade de Aveiro.
