import cv2
import os
import time
import pandas as pd


DATASET = "../dataset"
OUTPUT = "../results/hsv"

os.makedirs(OUTPUT, exist_ok=True)

rows = []

# AJUSTAR SE NECESSÁRIO
LOWER = (0, 0, 0)
UPPER = (180, 255, 70)


for scenario in os.listdir(DATASET):

    folder = os.path.join(DATASET, scenario)

    for img_name in os.listdir(folder):

        path = os.path.join(folder, img_name)

        frame = cv2.imread(path)

        start = time.time()

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        mask = cv2.inRange(
            hsv,
            LOWER,
            UPPER
        )

        kernel = (
            cv2.getStructuringElement(
                cv2.MORPH_RECT,
                (5, 5)
            )
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detectou = False

        bbox = [0, 0, 0, 0]

        if contours:

            detectou = True

            c = max(
                contours,
                key=cv2.contourArea
            )

            x, y, w, h = (
                cv2.boundingRect(c)
            )

            bbox = [x, y, w, h]

            cv2.rectangle(
                frame,
                (x, y),
                (x+w, y+h),
                (0,255,0),
                3
            )

        elapsed = (
            time.time()
            -
            start
        ) * 1000

        cv2.imwrite(
            f"{OUTPUT}/{scenario}_{img_name}",
            frame
        )

        rows.append({

            "image": img_name,
            "scenario": scenario,
            "detected": detectou,
            "confidence": 1 if detectou else 0,
            "time_ms": elapsed,

            "x": bbox[0],
            "y": bbox[1],
            "w": bbox[2],
            "h": bbox[3]

        })

pd.DataFrame(rows).to_csv(
    "../results/metrics/hsv.csv",
    index=False
)

print("HSV FINALIZADO")