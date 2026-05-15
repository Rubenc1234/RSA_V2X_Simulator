FROM python:3.9-slim

WORKDIR /app

# Instalar apenas dependências necessárias (não usar requirements.txt global)
RUN pip install paho-mqtt==2.1.0 requests

# Copiar simulator_core e cenários
COPY simulator_core.py .
COPY scenarios/ ./scenarios/

# Copiar script OBU2
COPY containers/obu2/obu2.py .

# Executar OBU2
CMD ["python", "obu2.py"]
