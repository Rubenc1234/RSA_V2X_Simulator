# Projeto V2X com Vanetza-NAP

## Objetivo

Este projeto pretende simular comunicação V2X (Vehicle-to-Everything) com ETSI C-ITS, usando containers Docker com Vanetza-NAP para representar veículos (OBUs) e uma RSU.

Objetivos funcionais alinhados com o enunciado:
- Criar varios containers em que cada container representa um veiculo.
- Gerar CAMs com base em localizacoes GPS emuladas.
- Visualizar as mensagens numa web app com mapa em tempo real.
- Injetar DENMs (acidente, emergencia, congestionamento) e modelar a reacao dos veiculos.

## Estado Atual do Repositorio

Ja existe:
- Infraestrutura Vanetza-NAP em `vanetza-nap/`.
- Orquestracao Docker em `vanetza-nap/docker-compose.yml`.
- Script `simulador.py` criado, mas ainda vazio.

Ainda nao existe (planeado):
- Web app (`web-app/`).
- Scripts de cenarios (`scenarios.py`) e trajetos (`trajetos.py`).
- Script unico de arranque (`run.sh`).

## Arquitetura (Alvo)

```
Python simulator (emulacao de trajetos e eventos)
  -> publica JSON CAM/DENM em MQTT (topics in)
     -> Vanetza codifica e envia por ITS-G5 na rede Docker
        -> outros containers recebem e descodificam
           -> publicacao MQTT (topics out)
              -> web app subscreve e atualiza mapa em tempo real
```

## Componentes

### 1) Containers Vanetza

Definidos em `vanetza-nap/docker-compose.yml`:
- `rsu` com Station ID 1
- `obu1` com Station ID 2
- `obu2` com Station ID 3

Todos incluem MQTT embebido e interface ITS-G5 simulada.

### 2) Simulador Python

Ficheiro atual: `simulador.py`.

Responsabilidades esperadas:
- Emular trajetos (lat/lon/speed/heading) de cada veiculo.
- Publicar CAM JSON para `vanetza/in/cam` no broker MQTT do container respetivo.
- Injetar DENM JSON para `vanetza/in/denm` em cenarios definidos.
- Aplicar regras de reacao (por exemplo, reduzir velocidade apos DENM relevante).

### 3) Web App (planeada)

Responsabilidades esperadas:
- Backend Node.js para consumir MQTT e expor stream em tempo real.
- Frontend com mapa (Leaflet) para mostrar OBUs e alertas DENM.

## Topicos MQTT Relevantes

Com base na configuracao default de `vanetza-nap/tools/socktap/config.ini`:
- CAM in: `vanetza/in/cam`
- CAM out: `vanetza/out/cam`
- DENM in: `vanetza/in/denm`
- DENM out: `vanetza/out/denm`

Nota:
- O fluxo por `vehicle/gps/...` pode ser usado noutras arquiteturas, mas neste repositorio o caminho mais direto e controlavel e publicar JSON ETSI nos topicos `vanetza/in/*`.

## Como Correr (Baseline Atual)

### Pre-requisitos

```bash
sudo apt update
sudo apt install -y docker.io docker-compose python3 python3-pip mosquitto-clients
pip install paho-mqtt
```

### 1) Criar rede Docker

```bash
docker network create vanetzalan0 --subnet 192.168.98.0/24
```

### 2) Levantar containers

```bash
cd vanetza-nap
docker-compose up -d
```

### 3) Verificar CAMs a sair

```bash
mosquitto_sub -h 192.168.98.20 -t "vanetza/out/cam" -v
```

Se os containers estiverem corretos, deves observar mensagens CAM periodicas.

## Ordem Recomendada de Implementacao

1. Validar baseline dos containers.
2. Implementar `simulador.py` para um unico veiculo (publicar CAM em `vanetza/in/cam`).
3. Expandir para varios veiculos e trajetos simples.
4. Implementar cenarios DENM (acidente, emergencia, congestionamento).
5. Adicionar regras de reacao dos veiculos no simulador.
6. Criar web app para visualizacao em mapa.
7. Instrumentar metricas (delivery rate, latencia, cobertura) para relatorio.

Porque esta ordem funciona bem:
- Reduz risco tecnico cedo (confirmas rede/Vanetza antes da UI).
- Permite demonstracoes incrementais ao stor.
- Se faltar tempo, ja tens nucleo funcional (CAM + DENM + reacao) mesmo sem UI final polida.

## Cenarios Sugeridos para Apresentacao

### Cenario 1: Acidente a frente
- RSU publica DENM de acidente numa coordenada.
- OBUs dentro de raio de influencia reduzem velocidade.

### Cenario 2: Veiculo de emergencia
- OBU de emergencia publica DENM prioritario.
- Veiculos na mesma via mudam comportamento (abrir corredor/simular desaceleracao).

### Cenario 3: Congestionamento
- RSU publica DENM de transito lento.
- OBUs passam para perfil de velocidade reduzida.

## Metricas para Relatorio

- Delivery rate: mensagens recebidas / mensagens enviadas.
- Latencia: diferenca entre timestamp de envio e de rececao.
- Cobertura: distancia maxima para rececao fiavel.

## Referencias

- Vanetza-NAP: https://github.com/nap-it/vanetza-nap
- ETSI C-ITS: https://www.etsi.org/
- MQTT: https://mqtt.org/
- Leaflet: https://leafletjs.com/

## Contexto Academico

Projeto da UC Redes e Sistemas Autonomos (RSA), Mestrado em Engenharia de Computadores e Telematica, Universidade de Aveiro.
