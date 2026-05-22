import cv2

from ultralytics import YOLO


class YoloTracker:

    def __init__(self, model_path):

        print("Carregando YOLO...")

        self.model = YOLO(model_path)

    def process_frame(self, frame):

        results = self.model(frame, verbose=False)

        annotated = results[0].plot()

        height, width, _ = frame.shape

        center_x = width // 2
        center_y = height // 2

        cv2.circle(
            annotated,
            (center_x, center_y),
            8,
            (255, 0, 0),
            -1
        )

        detections = results[0].boxes

        target_found = False

        error_x = 0
        error_y = 0

        if len(detections) > 0:

            target_found = True

            best_box = detections[0]

            x1, y1, x2, y2 = best_box.xyxy[0]

            target_x = int((x1 + x2) / 2)
            target_y = int((y1 + y2) / 2)

            error_x = target_x - center_x
            error_y = target_y - center_y

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

            cv2.putText(
                annotated,
                f"Erro X: {error_x}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            cv2.putText(
                annotated,
                f"Erro Y: {error_y}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

        return (
            annotated,
            target_found,
            error_x,
            error_y
        )