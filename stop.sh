#!/bin/bash

set -e

pkill -f mosquitto_sub >/dev/null 2>&1 || true
pkill -f "uvicorn backend:app" >/dev/null 2>&1 || true

docker-compose down --remove-orphans >/dev/null 2>&1 || true

if [ -f "vanetza-nap/docker-compose.yml" ]; then
	(cd vanetza-nap && docker-compose down) >/dev/null 2>&1 || true
fi
