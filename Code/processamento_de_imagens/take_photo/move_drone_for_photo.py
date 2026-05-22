import asyncio

from mavsdk import System
from mavsdk.offboard import (
    PositionNedYaw,
    OffboardError
)

# =========================================================
# CONFIG
# =========================================================

CYLINDER_X = 8.0
CYLINDER_Y = 5.0

DISTANCE_FROM_TARGET = 5.0

MOVEMENT_RADIUS = 10.0

ALTITUDES = [5.0, 10.0, 15.0]

MOVE_WAIT = 8

# =========================================================
# MOVE
# =========================================================

async def move_to(
    drone,
    north,
    east,
    down,
    yaw=0.0
):

    print(
        f"INDO PARA -> "
        f"N={north:.1f} "
        f"E={east:.1f} "
        f"D={down:.1f}"
    )

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            north,
            east,
            down,
            yaw
        )
    )

    await asyncio.sleep(MOVE_WAIT)


# =========================================================
# MAIN
# =========================================================

async def main():

    print("Conectando...")

    drone = System()

    await drone.connect(
        system_address="udp://:14550"
    )

    async for state in drone.core.connection_state():

        if state.is_connected:

            print("Drone conectado!")
            break

    # =====================================================
    # TAKEOFF NORMAL
    # =====================================================

    print("Configurando altitude...")

    await drone.action.set_takeoff_altitude(5.0)

    print("Armando...")

    await drone.action.arm()

    print("Takeoff...")

    await drone.action.takeoff()

    # MUITO IMPORTANTE
    print("Esperando estabilizar...")

    await asyncio.sleep(15)

    # =====================================================
    # OFFBOARD INIT
    # =====================================================

    print("Inicializando Offboard...")

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            0.0,
            0.0,
            -5.0,
            0.0
        )
    )

    try:

        await drone.offboard.start()

    except OffboardError as e:

        print(f"Erro Offboard: {e}")

        await drone.action.disarm()

        return

    # =====================================================
    # BASE
    # =====================================================

    base_x = CYLINDER_X - DISTANCE_FROM_TARGET
    base_y = CYLINDER_Y

    print(
        f"Base -> "
        f"X={base_x} "
        f"Y={base_y}"
    )

    # =====================================================
    # ALTURAS
    # =====================================================

    for altitude in ALTITUDES:

        print(f"\nALTURA = {altitude}m\n")

        down = -altitude

        # CENTRO
        await move_to(
            drone,
            base_x,
            base_y,
            down
        )

        # FRENTE
        await move_to(
            drone,
            base_x + MOVEMENT_RADIUS,
            base_y,
            down
        )

        # TRAS
        await move_to(
            drone,
            base_x - MOVEMENT_RADIUS,
            base_y,
            down
        )

        # CENTRO
        await move_to(
            drone,
            base_x,
            base_y,
            down
        )

        # DIREITA
        await move_to(
            drone,
            base_x,
            base_y + MOVEMENT_RADIUS,
            down
        )

        # ESQUERDA
        await move_to(
            drone,
            base_x,
            base_y - MOVEMENT_RADIUS,
            down
        )

        # CENTRO
        await move_to(
            drone,
            base_x,
            base_y,
            down
        )

    # =====================================================
    # FINAL
    # =====================================================

    print("Pousando...")

    await drone.action.land()

    await asyncio.sleep(10)

    print("Fim!")


# =========================================================
# START
# =========================================================

if __name__ == "__main__":

    asyncio.run(main())