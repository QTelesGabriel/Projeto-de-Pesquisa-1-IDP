import asyncio
import time

import cv2
import numpy as np

from mavsdk.offboard import (
    VelocityBodyYawspeed
)

# =========================================================
# PID CONFIG
# =========================================================

KP = 0.002
KI = 0.0
KD = 0.0001

MAX_SPEED = 0.3

DEADZONE = 10


async def precision_land(
    drone,
    tracker,
    node
):

    print("Iniciando precision landing...")

    previous_error_x = 0
    previous_error_y = 0

    integral_x = 0
    integral_y = 0

    previous_time = time.time()

    cv2.namedWindow(
        "YOLO FOLLOW",
        cv2.WINDOW_NORMAL
    )

    while True:

        await asyncio.sleep(0.03)

        if node.frame is None:
            continue

        frame = node.frame.copy()

        (
            annotated,
            target_found,
            error_x,
            error_y
        ) = tracker.process_frame(frame)

        forward_speed = 0.0
        right_speed = 0.0

        if target_found:

            current_time = time.time()

            dt = current_time - previous_time

            previous_time = current_time

            if dt <= 0:
                dt = 0.001

            # =============================================
            # DEADZONE
            # =============================================

            current_kd = KD

            if abs(error_x) < DEADZONE and abs(error_y) < DEADZONE:
                current_kd = 0

            # =============================================
            # INTEGRAL
            # =============================================

            integral_x += error_x * dt
            integral_y += error_y * dt

            # =============================================
            # DERIVATIVO
            # =============================================

            derivative_x = (
                error_x - previous_error_x
            ) / dt

            derivative_y = (
                error_y - previous_error_y
            ) / dt

            previous_error_x = error_x
            previous_error_y = error_y

            # =============================================
            # PID
            # =============================================

            right_speed = (
                KP * error_x
                + KI * integral_x
                + current_kd * derivative_x
            )

            forward_speed = -(
                KP * error_y
                + KI * integral_y
                + current_kd * derivative_y
            )

            # =============================================
            # LIMITES
            # =============================================

            right_speed = np.clip(
                right_speed,
                -MAX_SPEED,
                MAX_SPEED
            )

            forward_speed = np.clip(
                forward_speed,
                -MAX_SPEED,
                MAX_SPEED
            )

            print(
                f"ErroX={error_x} "
                f"ErroY={error_y}"
            )

            print(
                f"Forward={forward_speed:.2f} "
                f"Right={right_speed:.2f}"
            )

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                forward_m_s=float(forward_speed),
                right_m_s=float(right_speed),
                down_m_s=0.0,
                yawspeed_deg_s=0.0
            )
        )

        cv2.imshow(
            "YOLO FOLLOW",
            annotated
        )

        key = cv2.waitKey(1)

        if key == ord('q'):
            break

    cv2.destroyAllWindows()