from pymavlink import mavutil

def enviar_velocidade(vehicle, velocity_x, velocity_y, velocity_z, yaw_rate=0.0):
    """
    Move o veículo especificando velocidades (m/s) e taxa de rotação (rad/s).
    velocity_x: positivo = frente / negativo = trás
    velocity_y: positivo = direita / negativo = esquerda
    velocity_z: positivo = baixo (descida) / negativo = cima (subida)
    yaw_rate: positivo = gira direita (horário) / negativo = gira esquerda (anti-horário)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (não usado)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # Referencial: Baseado na frente do drone
        0b0000011111000111, # Máscara: Usa velocidade e Yaw Rate (para travar ou girar o eixo Z)
        0, 0, 0, # Posições X, Y, Z (ignoradas)
        velocity_x, velocity_y, velocity_z, # Velocidades X, Y, Z (m/s)
        0, 0, 0, # Acelerações (ignoradas)
        0, yaw_rate)    # Yaw (ignorado), Yaw rate
        
    vehicle.send_mavlink(msg)
    vehicle.flush()

def ir_para_posicao_local(vehicle, frente_x, direita_y, baixo_z):
    """
    Move o drone para uma posição X, Y, Z (em metros) relativa ao ponto de decolagem.
    Referencial LOCAL_NED:
    frente_x: Positivo é Norte (Frente no Gazebo)
    direita_y: Positivo é Leste (Direita no Gazebo)
    baixo_z: Positivo é para BAIXO, negativo é para CIMA (Ex: -5.0 sobe 5m)
    """
    # 0b0000111111111000 = Máscara para ignorar velocidade e aceleração, usando só posição
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b0000111111111000, 
        frente_x, direita_y, baixo_z, 
        0, 0, 0, # Velocidades (ignoradas)
        0, 0, 0, # Acelerações (ignoradas)
        0, 0)    # Yaw e Yaw rate
        
    vehicle.send_mavlink(msg)
    vehicle.flush()