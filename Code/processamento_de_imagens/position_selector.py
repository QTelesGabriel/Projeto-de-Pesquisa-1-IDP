import asyncio

from mavsdk.offboard import (
    PositionNedYaw
)

# ==========================================
# CENTRO DA PLATAFORMA
# ==========================================

TARGET_X = 8.0
TARGET_Y = 5.0


async def select_test_position(
    drone,
    altitude
):

    radius = altitude

    positions = {

        1: (0, radius),
        2: (radius, radius),
        3: (radius, 0),
        4: (radius, -radius),
        5: (0, -radius),
        6: (-radius, -radius),
        7: (-radius, 0),
        8: (-radius, radius)
    }

    print("\n========== POSICOES ==========")

    print("1 - Norte")
    print("2 - Nordeste")
    print("3 - Leste")
    print("4 - Sudeste")
    print("5 - Sul")
    print("6 - Sudoeste")
    print("7 - Oeste")
    print("8 - Noroeste")

    option = int(
        input(
            "\nEscolha a posicao: "
        )
    )

    dx, dy = positions[option]

    target_x = TARGET_X + dx
    target_y = TARGET_Y + dy

    print(
        f"\nIndo para "
        f"({target_x:.2f}, "
        f"{target_y:.2f})"
    )

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            north_m=target_x,
            east_m=target_y,
            down_m=-altitude,
            yaw_deg=0
        )
    )

    await asyncio.sleep(10)

    print("Posicao alcançada")