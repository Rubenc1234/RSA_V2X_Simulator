FROM python:3.9-slim

WORKDIR /app

RUN pip install paho-mqtt==2.1.0 requests

COPY simulator_core.py .
COPY scenarios/ ./scenarios/
COPY agents/ ./agents/
COPY containers/obu1/obu1.py .

CMD ["python", "obu1.py"]