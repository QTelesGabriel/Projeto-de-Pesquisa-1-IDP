import time
from dronekit import VehicleMode

def arm_and_takeoff(vehicle, target_altitude):
    """
    Prepara o drone, muda para o modo GUIDED, arma os motores e decola.
    """
    print("Verificando se o drone pode ser armado...")
    # vehicle.is_armable garante que o GPS fixou e o giroscópio calibrou
    while not vehicle.is_armable:
        print(" Aguardando inicialização dos sensores do drone...")
        time.sleep(1)

    print("Mudando para o modo GUIDED...")
    # GUIDED é o modo exigido pelo ArduPilot para controle autônomo via código
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != 'GUIDED':
        print(" Aguardando mudança de modo...")
        time.sleep(1)

    print("Armando os motores...")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Aguardando os motores armarem...")
        time.sleep(1)

    print("Motores armados! Iniciando decolagem...")
    # Envia o comando de decolagem
    vehicle.simple_takeoff(target_altitude)

    # Loop para monitorar a altitude e não travar o restante do código
    while True:
        # Pega a altitude relativa ao ponto de decolagem
        altitude_atual = vehicle.location.global_relative_frame.alt
        print(f" Altitude atual: {altitude_atual:.2f} metros")
        
        # Consideramos sucesso ao atingir 95% da altitude alvo
        if altitude_atual >= target_altitude * 0.95:
            print("Altitude alvo alcançada com sucesso!")
            break
        time.sleep(1)