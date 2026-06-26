FROM python:3.9-slim

WORKDIR /app

# Instala as dependências necessárias
RUN pip install --no-cache-dir paho-mqtt==2.1.0 requests

# Copia a estrutura respeitando os módulos do Python
COPY domain/ ./domain/
COPY scenarios/ ./scenarios/
COPY agents/ ./agents/

# Define o PYTHONPATH para o Python conseguir encontrar os módulos
ENV PYTHONPATH=/app

# O ponto de entrada agora é o teu bootstrap genérico
CMD ["python", "agents/obu_main_v2.py"]
