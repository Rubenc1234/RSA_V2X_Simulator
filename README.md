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
pip install paho-mqtt fastapi uvicorn 'uvicorn[standard]'
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
   ./start.sh
   ```
   
   Abrir http://localhost:8000 no browser.
   Depois de 30-60 segundos,
   
   Para parar tudo:
   ```bash
   ./stop.sh
   ```

6. **Opcao B - Manual**

   Terminal 1 - Arrancar simulador:
   ```bash
   python3 simulador.py
   ```

   Terminal 2 - Observar CAMs:
   ```bash
   mosquitto_sub -h 192.168.98.20 -t vanetza/out/cam -v
   ```

   Terminal 3 - Arrancar web app:
   ```bash
   uvicorn backend:app --host 0.0.0.0 --port 8000
   ```

   Depois abrir http://localhost:8000 no browser.

7. Analisar logs (depois de parar o simulador):
   ```bash
   python3 analyze_logs.py
   ```

## Analise de Logs

Depois de executar o simulador, podes analisar os dados recolhidos:

```bash
python3 analyze_logs.py
```

O script `analyze_logs.py`:
- Lê logs guardados em `logs/obu1_cam_log.jsonl` e `logs/obu2_cam_log.jsonl`
- Compara posicoes com as rotas esperadas (definidas em `simulador.py`)
- Mostra estatisticas:
  - Distancia media ao trajeto esperado
  - Desvio maximo
  - Velocidades e headings registados
  - Cobertura de waypoints

Exemplo de output:
```
 Analysis for OBU1
 Total CAM messages: 210
--- Route Accuracy ---
  ✓ Average distance to route: 7.67 m
  ✓ Maximum deviation: 22.92 m
  ✅ Good: Position follows route
```

## Componentes Principais

### simulador.py
O script [simulador.py](simulador.py) implementa um MVP de mobilidade para duas OBUs:
- Define 2 trajetos GPS (listas de pontos).
- Interpola posicao entre segmentos para movimento continuo.
- Calcula heading por segmento.
- Publica CAM a cada tick (5 Hz) em cada broker OBU:
   - OBU1 em 192.168.98.20 (stationId 2 no compose)
   - OBU2 em 192.168.98.21 (stationId 3 no compose)

Payload CAM:
- Estrutura minima valida alinhada com os exemplos do Vanetza.
- Inclui basicContainer (posicao) e highFrequencyContainer (heading, speed, etc.).

### backend.py
O script [backend.py](backend.py) implementa um agregador MQTT + mapa web:
- Subscreve aos 3 brokers Vanetza em `vanetza/out/cam` e `vanetza/out/denm`
- Normaliza mensagens e reemite por WebSocket
- Serve mapa Leaflet em tempo real em `http://localhost:8000`
- Mostra 2 marcadores a moverem de acordo com CAMs recebidos

### analyze_logs.py
O script [analyze_logs.py](analyze_logs.py) valida os dados recolhidos:
- Compara posicoes com as rotas esperadas
- Calcula desvio medio e maximo
- Mostra estatisticas de velocidade e heading
- Gera relatorio JSON em `logs/`

## Proximas Etapas

Depois de validar o MVP CAM:
1. Adicionar injecao DENM por cenarios (acidente, emergencia, congestionamento).
2. Implementar reacao por veiculo com base em distancia e validade do alerta.
3. Criar backend + frontend para mapa em tempo real.
4. Medir indicadores para relatorio (delivery rate, latencia, cobertura).

## Referencias

- Vanetza-NAP: https://github.com/nap-it/vanetza-nap
- ETSI C-ITS: https://www.etsi.org/
- MQTT: https://mqtt.org/
- Leaflet: https://leafletjs.com/

## Contexto Academico

Projeto da UC Redes e Sistemas Autonomos (RSA), Mestrado em Engenharia de Computadores e Telematica, Universidade de Aveiro.
