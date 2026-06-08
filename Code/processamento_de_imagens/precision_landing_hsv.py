import asyncio
import time

import cv2
import numpy as np

from mavsdk.offboard import (
    VelocityBodyYawspeed
)

KP = 0.002
KD = 0.0001

MAX_SPEED = 0.3

DEADZONE = 10


async def precision_land(
    drone,
    tracker,
    node
):

    print(
        "Precision Landing iniciado"
    )

    previous_error_x = 0
    previous_error_y = 0

    previous_time = time.time()

    aligned_counter = 0

    cv2.namedWindow(
        "Camera",
        cv2.WINDOW_NORMAL
    )

    cv2.namedWindow(
        "HSV Mask",
        cv2.WINDOW_NORMAL
    )

    cv2.namedWindow(
        "Precision Landing",
        cv2.WINDOW_NORMAL
    )

    while True:

        await asyncio.sleep(0.03)

        if node.frame is None:
            continue

        (
            frame,
            mask,
            annotated,
            target_found,
            error_x,
            error_y
        ) = tracker.process_frame(
            node.frame.copy()
        )

        print(
            f"TARGET={target_found} "
            f"ERRX={error_x} "
            f"ERRY={error_y}"
        )

        forward_speed = 0.0
        right_speed = 0.0
        down_speed = 0.0

        if target_found:

            current_time = time.time()

            dt = (
                current_time
                - previous_time
            )

            previous_time = current_time

            if dt <= 0:
                dt = 0.001

            derivative_x = (
                error_x -
                previous_error_x
            ) / dt

            derivative_y = (
                error_y -
                previous_error_y
            ) / dt

            previous_error_x = error_x
            previous_error_y = error_y

            right_speed = (
                KP * error_x +
                KD * derivative_x
            )

            forward_speed = -(
                KP * error_y +
                KD * derivative_y
            )

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

            cv2.putText(
                annotated,
                f"ErroX: {error_x}",
                (20,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0,255,255),
                2
            )

            cv2.putText(
                annotated,
                f"ErroY: {error_y}",
                (20,80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0,255,255),
                2
            )

            if (
                abs(error_x) < DEADZONE
                and
                abs(error_y) < DEADZONE
            ):

                aligned_counter += 1

                down_speed = 0.15

                cv2.putText(
                    annotated,
                    "DESCENDO",
                    (20,140),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,255),
                    2
                )

            else:

                aligned_counter = 0

        else:

            cv2.putText(
                annotated,
                "ALVO NAO ENCONTRADO",
                (20,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0,0,255),
                2
            )

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                float(forward_speed),
                float(right_speed),
                float(down_speed),
                0.0
            )
        )

        cv2.imshow(
            "Camera",
            frame
        )

        cv2.imshow(
            "HSV Mask",
            mask
        )

        cv2.imshow(
            "Precision Landing",
            annotated
        )

        key = cv2.waitKey(1)

        if key == ord("q"):
            break

        if aligned_counter > 100:

            print(
                "Alinhado. Pousando."
            )

            await drone.action.land()

            break

    cv2.destroyAllWindows()