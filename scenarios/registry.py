"""Scenario registry for the V2X simulator."""

from __future__ import annotations

import os
from typing import Dict, List

SCENARIOS: Dict[str, dict] = {
	"1": {
		"name": "arnaco_braga",
		"label": "R. Arnaçó, Braga",
		"mapCenter": [41.726350, -8.164850],
		"mapZoom": 17,
		"brokers": [
			{"host": "192.168.98.20", "name": "obu1", "stationId": 2},
			{"host": "192.168.98.21", "name": "obu2", "stationId": 3},
			{"host": "192.168.98.22", "name": "obu3", "stationId": 4},
			{"host": "192.168.98.23", "name": "obu4", "stationId": 5},
			{"host": "192.168.98.24", "name": "obu5", "stationId": 6},
			{"host": "192.168.98.25", "name": "obu6", "stationId": 7},
		],
		"vehicles": [
			{
				"name": "obu1",
				"stationId": 2,
				"broker": "192.168.98.20",
				"startPoint": [41.727849, -8.163264],
				"endPoint": [41.725639, -8.165512],
				"baseSpeedMps": 8.0,
			},
			{
				"name": "obu2",
				"stationId": 3,
				"broker": "192.168.98.21",
				"startPoint": [41.725639, -8.165512],
				"endPoint": [41.727849, -8.163264],
				"baseSpeedMps": 9.0,
			},
			{
				"name": "obu3",
				"stationId": 4,
				"broker": "192.168.98.22",
				"startPoint": [41.728674, -8.162331],
				"endPoint": [41.725775, -8.163500],
				"baseSpeedMps": 8.5,
			},
			{
				"name": "obu4",
				"stationId": 5,
				"broker": "192.168.98.23",
				"startPoint": [41.727329, -8.165066],
				"endPoint": [41.724678, -8.165206],
				"baseSpeedMps": 8.0,
			},
			{
				"name": "obu5",
				"stationId": 6,
				"broker": "192.168.98.24",
				"startPoint": [41.728129, -8.165324],
				"endPoint": [41.724918, -8.165238],
				"baseSpeedMps": 8.2,
			},
			{
				"name": "obu6",
				"stationId": 7,
				"broker": "192.168.98.25",
				"startPoint": [41.728377, -8.162974],
				"endPoint": [41.725599, -8.163114],
				"baseSpeedMps": 8.3,
			},
		],
	},
	"2": {
		"name": "intersection_rsu",
		"label": "RSU Intersection",
		"mapCenter": [41.549800, -8.428100],
		"mapZoom": 17,
		"brokers": [
			{"host": "192.168.98.10", "name": "rsu", "stationId": 1},
			{"host": "192.168.98.20", "name": "obu1", "stationId": 2},
			{"host": "192.168.98.21", "name": "obu2", "stationId": 3},
			{"host": "192.168.98.22", "name": "obu3", "stationId": 4},
			{"host": "192.168.98.23", "name": "obu4", "stationId": 5},
		],
	},
	"3": {
		"name": "boavista_accident",
		"label": "Porto - Rotunda da Boavista (Accident & Reroute)",
		"mapCenter": [41.160300, -8.629900],
		"mapZoom": 16,
		"brokers": [
			{"host": "192.168.98.20", "name": "obu1", "stationId": 2},
			{"host": "192.168.98.21", "name": "obu2", "stationId": 3},
		],
		"vehicles": [
			{
				"name": "obu1",
				"stationId": 2,
				"broker": "192.168.98.20",
				"startPoint": [41.166948, -8.653663],
				"endPoint": [41.167117, -8.654610],
				"baseSpeedMps": 8.0,
				"loopRoute": False,
				"emitCollisionRiskDenm": False,
				"incidentOnArrival": True,
				"incidentCauseCode": 2,
				"incidentSubCauseCode": 0,
				"incidentValidityDuration": 20,
			},
			{
				"name": "obu2",
				"stationId": 3,
				"broker": "192.168.98.21",
				"startPoint": [41.166889, -8.653291],
				"endPoint": [41.167249, -8.655549],
				"baseSpeedMps": 9.0,
			},
		],
	},
}

# not used
def get_scenario_name(default: str = "1") -> str:
	"""Resolve the active scenario from the environment."""
	return os.getenv("SIM_SCENARIO", default)


def get_scenario_config(name: str | None = None) -> dict:
	"""Return the config for one scenario, falling back to the default."""
	resolved_name = name or get_scenario_name()
	return SCENARIOS.get(resolved_name, SCENARIOS["1"])


def get_vehicle_config(vehicle_name: str, scenario_name: str | None = None, default: dict | None = None) -> dict:
	"""Return the vehicle config for the active scenario, falling back to default."""
	scenario = get_scenario_config(scenario_name)
	for vehicle in scenario.get("vehicles", []):
		if vehicle.get("name") == vehicle_name:
			return vehicle
	return default or {}


def get_scenario_vehicle_names(name: str | None = None) -> List[str]:
	"""Return the vehicle names declared by the active scenario."""
	scenario = get_scenario_config(name)
	return [vehicle.get("name", "") for vehicle in scenario.get("vehicles", []) if vehicle.get("name")]


def list_scenarios() -> List[dict]:
	"""Return all scenario configs."""
	return list(SCENARIOS.values())
