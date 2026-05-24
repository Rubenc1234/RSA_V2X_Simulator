"""Thin bootstrap for an OBU that dynamically uses the right agent based on the scenario."""

import os
from scenarios.registry import get_vehicle_config, get_scenario_config
from agents.car_agent import CarAgent
from agents.lane_change_agent import LaneChangeAgent

def main():
    scenario_id = os.environ.get("SIM_SCENARIO", "1")
    
    # Use VEHICLE_NAME if available, fallback to MQTT_BROKER name (both are usually e.g., "obu3")
    vehicle_name = os.environ.get("VEHICLE_NAME", os.environ.get("MQTT_BROKER", "obu1"))
    
    print(f"[{vehicle_name}] Booting Scenario {scenario_id}...")
    
    # Fetch the vehicle's specific configuration from registry.py
    profile = get_vehicle_config(vehicle_name)

    # Route to the correct agent and pass the profile!
    if scenario_id == "2":
        agent = LaneChangeAgent(profile)
    else:
        agent = CarAgent(profile)

    # Attach peer brokers declared in scenario for CAM/DENM subscription
    scenario = get_scenario_config()
    for broker in scenario.get("brokers", []):
        host = broker.get("host")
        name = broker.get("name")
        if host and host != agent.broker:
            agent.attach_peer_listener(host, f"{vehicle_name}-peer-{name}")

    agent.run()

if __name__ == "__main__":
    main()