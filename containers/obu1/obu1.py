"""Thin bootstrap for an OBU that uses the shared CarAgent implementation."""

import os

from scenarios.registry import get_vehicle_config, get_scenario_config
from agents.car_agent import CarAgent


def main():
    vehicle_name = os.environ.get("VEHICLE_NAME", "obu1")
    profile = get_vehicle_config(vehicle_name)
    agent = CarAgent(profile)

    # attach peer brokers declared in scenario for CAM/DENM subscription
    scenario = get_scenario_config()
    for broker in scenario.get("brokers", []):
        host = broker.get("host")
        name = broker.get("name")
        if host and host != agent.broker:
            agent.attach_peer_listener(host, f"{vehicle_name}-peer-{name}")

    agent.run()


if __name__ == "__main__":
    main()
