import asyncio

from mavsdk import System


TAKEOFF_ALTITUDE_M = 10.0
ALTITUDE_TOLERANCE_M = 0.5
TAKEOFF_TIMEOUT_S = 45.0


async def _wait_until_takeoff_altitude(drone):
    async for position in drone.telemetry.position():
        altitude = position.relative_altitude_m

        print(
            f"Altitude: {altitude:.1f} m / "
            f"{TAKEOFF_ALTITUDE_M:.1f} m"
        )

        if altitude >= TAKEOFF_ALTITUDE_M - ALTITUDE_TOLERANCE_M:
            return


async def takeoff(drone):

    print("Configurando takeoff...")

    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)

    print("Armando drone...")

    await drone.action.arm()

    print("Decolando...")

    await drone.action.takeoff()

    try:
        await asyncio.wait_for(
            _wait_until_takeoff_altitude(drone),
            timeout=TAKEOFF_TIMEOUT_S,
        )
    except asyncio.TimeoutError as error:
        raise RuntimeError(
            "O drone nao atingiu a altitude de takeoff dentro do tempo limite."
        ) from error

    print("Altitude atingida. Mantendo posicao...")

    # O modo HOLD mantem latitude, longitude, altitude e direcao atuais.
    await drone.action.hold()

    print("Takeoff concluido! Drone parado em HOLD.")
