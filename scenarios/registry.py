"""Scenario registry for the V2X simulator."""

from __future__ import annotations

import os
from typing import Dict, List

SCENARIOS: Dict[str, dict] = {
	"1": {
		"name": "arnaco_braga",
		"label": "R. Arnaçó, Braga",
		"module": "scenarios.arnaco_braga",
		"mapCenter": [41.726350, -8.164850],
		"mapZoom": 17,
		"brokers": [
			{"host": "192.168.98.20", "name": "obu1"},
			{"host": "192.168.98.21", "name": "obu2"},
		],
	},
	"2": {
		"name": "intersection_rsu",
		"label": "RSU Intersection",
		"module": "scenarios.intersection_rsu",
		"mapCenter": [41.549800, -8.428100],
		"mapZoom": 17,
		"brokers": [
			{"host": "192.168.98.10", "name": "rsu"},
			{"host": "192.168.98.20", "name": "obu1"},
			{"host": "192.168.98.21", "name": "obu2"},
			{"host": "192.168.98.22", "name": "obu3"},
			{"host": "192.168.98.23", "name": "obu4"},
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


def list_scenarios() -> List[dict]:
	"""Return all scenario configs."""
	return list(SCENARIOS.values())
