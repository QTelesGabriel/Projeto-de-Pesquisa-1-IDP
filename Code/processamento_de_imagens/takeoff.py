import asyncio


async def takeoff(drone):

    print("Armando")

    await drone.action.set_takeoff_altitude(
        10
    )

    await drone.action.arm()

    print("Decolando")

    await drone.action.takeoff()

    await asyncio.sleep(20)

    print(
        "Takeoff concluido"
    )