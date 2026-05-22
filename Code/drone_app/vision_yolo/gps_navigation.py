import asyncio
import random
import math

from mavsdk.telemetry import LandedState


# =========================================================
# CONFIGURACAO
# =========================================================

TARGET_LAT = -35.363802
TARGET_LON = 149.166119

GPS_NOISE_METERS = 10.0

ARRIVAL_THRESHOLD_METERS = 3.0


# =========================================================
# CONVERSAO METROS -> GRAUS
# =========================================================

def meters_to_lat(meters):

    return meters / 111111.0


def meters_to_lon(meters, latitude):

    return meters / (
        111111.0 * math.cos(math.radians(latitude))
    )


# =========================================================
# DISTANCIA GPS
# =========================================================

def haversine_distance(
    lat1,
    lon1,
    lat2,
    lon2
):

    R = 6371000.0

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)

    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(dlon / 2) ** 2
    )

    c = 2 * math.atan2(
        math.sqrt(a),
        math.sqrt(1 - a)
    )

    return R * c


# =========================================================
# GPS NAVIGATION
# =========================================================

async def gps_navigation(drone):

    print("Lendo posicao atual...")

    async for position in drone.telemetry.position():

        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        current_alt = position.absolute_altitude_m

        break

    print(
        f"POSICAO ATUAL:\n"
        f"LAT={current_lat}\n"
        f"LON={current_lon}\n"
        f"ALT={current_alt}"
    )

    # =====================================================
    # RUIDO RANDOMICO
    # =====================================================

    noise_x = random.uniform(
        -GPS_NOISE_METERS,
        GPS_NOISE_METERS
    )

    noise_y = random.uniform(
        -GPS_NOISE_METERS,
        GPS_NOISE_METERS
    )

    noisy_lat = (
        TARGET_LAT
        + meters_to_lat(noise_y)
    )

    noisy_lon = (
        TARGET_LON
        + meters_to_lon(
            noise_x,
            TARGET_LAT
        )
    )

    print(
        "\nAPLICANDO RUIDO GPS:"
    )

    print(
        f"ERRO X = {noise_x:.2f} metros"
    )

    print(
        f"ERRO Y = {noise_y:.2f} metros"
    )

    print(
        f"\nINDO PARA:"
    )

    print(
        f"LAT={noisy_lat}"
    )

    print(
        f"LON={noisy_lon}"
    )

    print(
        f"ALT={current_alt}"
    )

    # =====================================================
    # GOTO
    # =====================================================

    await drone.action.goto_location(
        noisy_lat,
        noisy_lon,
        current_alt,
        0
    )

    # =====================================================
    # ESPERAR CHEGADA
    # =====================================================

    print("\nNavegando...")

    while True:

        async for position in drone.telemetry.position():

            distance = haversine_distance(
                position.latitude_deg,
                position.longitude_deg,
                noisy_lat,
                noisy_lon
            )

            print(
                f"Distancia restante: "
                f"{distance:.2f} m"
            )

            if distance < ARRIVAL_THRESHOLD_METERS:

                print(
                    "\nDestino alcançado!"
                )

                return

            break

        await asyncio.sleep(1)