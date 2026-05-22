import asyncio

from mavsdk import System


async def takeoff(drone):

    print("Configurando takeoff...")

    await drone.action.set_takeoff_altitude(10.0)

    print("Armando drone...")

    await drone.action.arm()

    print("Decolando...")

    await drone.action.takeoff()

    await asyncio.sleep(20)

    print("Takeoff concluido!")