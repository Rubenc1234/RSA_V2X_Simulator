"""Entry point for the V2X simulator."""

from __future__ import annotations

import argparse
import importlib

from scenarios.registry import get_scenario_config, get_scenario_name

# criar configs para cada cenário dinamicamente
def main() -> None:
	parser = argparse.ArgumentParser(description="Run a V2X simulation scenario")
	parser.add_argument(
		"--scenario",
		help="Scenario name from scenarios/registry.py",
		default=None,
	)
	args = parser.parse_args()
	scenario_name = args.scenario #or get_scenario_name()
	scenario = get_scenario_config(scenario_name)
	# importar o cenário certi dinamicamente
	module = importlib.import_module(scenario["module"])
	print(f"Starting scenario: {scenario['name']} ({scenario['label']})")
	module.run()

if __name__ == "__main__":
	main()
