FROM python:3.9-slim

WORKDIR /app

# Instalar apenas dependências necessárias (não usar requirements.txt global)
RUN pip install paho-mqtt==2.1.0 requests

# Copiar script RSU
COPY containers/rsu/rsu.py .

# Executar RSU
CMD ["python", "rsu.py"]
