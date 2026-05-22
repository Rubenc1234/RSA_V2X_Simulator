"""Thin bootstrap for OBU1 that uses the shared CarAgent implementation."""

from scenarios.registry import get_vehicle_config, get_scenario_config
from agents.car_agent import CarAgent


def main():
    profile = get_vehicle_config("obu1")
    agent = CarAgent(profile)

    # attach peer brokers declared in scenario for CAM/DENM subscription
    scenario = get_scenario_config()
    for broker in scenario.get("brokers", []):
        host = broker.get("host")
        name = broker.get("name")
        if host and host != agent.broker:
            agent.attach_peer_listener(host, f"obu1-peer-{name}")

    agent.run()


if __name__ == "__main__":
    main()
