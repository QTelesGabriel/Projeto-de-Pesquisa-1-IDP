import cv2
import numpy as np


class HSVTracker:

    def __init__(self):

        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])

        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

    def process_frame(self, frame):

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        mask1 = cv2.inRange(
            hsv,
            self.lower_red1,
            self.upper_red1
        )

        mask2 = cv2.inRange(
            hsv,
            self.lower_red2,
            self.upper_red2
        )

        mask = cv2.bitwise_or(
            mask1,
            mask2
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        annotated = frame.copy()

        h, w, _ = frame.shape

        center_x = w // 2
        center_y = h // 2

        cv2.circle(
            annotated,
            (center_x, center_y),
            8,
            (255, 0, 0),
            -1
        )

        target_found = False
        error_x = 0
        error_y = 0

        if contours:

            largest = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(
                largest
            )

            if area > 100:

                x, y, bw, bh = cv2.boundingRect(
                    largest
                )

                target_x = x + bw // 2
                target_y = y + bh // 2

                error_x = (
                    target_x - center_x
                )

                error_y = (
                    target_y - center_y
                )

                target_found = True

                cv2.rectangle(
                    annotated,
                    (x, y),
                    (x + bw, y + bh),
                    (0, 255, 0),
                    2
                )

                cv2.circle(
                    annotated,
                    (target_x, target_y),
                    8,
                    (0, 255, 0),
                    -1
                )

                cv2.line(
                    annotated,
                    (center_x, center_y),
                    (target_x, target_y),
                    (0, 255, 255),
                    2
                )

        return (
            frame,
            mask,
            annotated,
            target_found,
            error_x,
            error_y
        )