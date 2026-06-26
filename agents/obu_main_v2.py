# obu_main_v2.py
import os
from scenarios.registry_2 import get_scenario_config
from agents.car_agent_v2 import DecentralizedCarAgent

def main():
    # Identifica quem ele é através das variáveis do docker-compose.yml
    scenario_id = os.environ.get("SIM_SCENARIO", "collision_risk")
    vehicle_name = os.environ.get("VEHICLE_NAME", "obu1")
    
    print(f"[{vehicle_name}] A iniciar em modo 100% DESCENTRALIZADO no cenário '{scenario_id}'...")
    
    # Carrega as configurações globais do registo comum
    scenario_config = get_scenario_config(scenario_id)
    
    # Filtra e extrai apenas a configuração específica deste veículo
    my_config = next((v for v in scenario_config["vehicles"] if v["name"] == vehicle_name), None)
    
    if not my_config:
        print(f"[Erro] Não foi encontrada configuração para o veículo {vehicle_name} no cenário {scenario_id}.")
        return
    
    print(f"[{vehicle_name}] Config carregada: {my_config}")
    print(f"[{vehicle_name}] Start: {my_config['startPoint']}, End: {my_config['endPoint']}")

    # Instancia o novo agente descentralizado e executa
    print("\n=== DIAGNOSTIC ===")
    print(f"Vehicle Config: {my_config}")
    agent = DecentralizedCarAgent(vehicle_name, my_config, scenario_config)
    print(f"Vehicle posição após init: ({agent.my_vehicle.current_lat}, {agent.my_vehicle.current_lon})")
    print(f"Vehicle waypoints: {len(agent.my_vehicle.waypoints)} pontos")
    print(f"First waypoint: {agent.my_vehicle.waypoints[0] if agent.my_vehicle.waypoints else 'NENHUM'}")
    print(f"Route from OSRM: {len(agent.route)} pontos")
    print(f"Route primeira: {agent.route[0] if agent.route else 'NENHUM'}")
    print("=================\n")
    
    print(f"[{vehicle_name}] Vehicle inicializado: lat={agent.my_vehicle.current_lat}, lon={agent.my_vehicle.current_lon}")
    print(f"[{vehicle_name}] Route carregada: {agent.route}")

    for broker_data in scenario_config.get("brokers", []):
        host = broker_data.get("host")
        name = broker_data.get("name")
        if host and host != agent.broker:
            agent.attach_peer_listener(host, f"{vehicle_name}-peer-{name}")

    print(f"[{vehicle_name}] Inicialização completa. A iniciar loops...")
    
    agent.run()

if __name__ == "__main__":
    main()
