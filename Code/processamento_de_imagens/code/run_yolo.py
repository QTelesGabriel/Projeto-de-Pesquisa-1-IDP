import cv2
import os
import time
import pandas as pd

from ultralytics import YOLO


MODEL = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/drone_app/vision_yolo/train/weights/best.pt"

model = YOLO(MODEL)

DATASET = "../dataset"

OUTPUT = "../results/yolo"

os.makedirs(
    OUTPUT,
    exist_ok=True
)

rows = []


for scenario in os.listdir(DATASET):

    folder = (
        f"{DATASET}/{scenario}"
    )

    for img_name in os.listdir(folder):

        path = (
            f"{folder}/{img_name}"
        )

        frame = cv2.imread(path)

        start = time.time()

        results = model(
            frame,
            verbose=False
        )

        elapsed = (
            time.time()
            -
            start
        )*1000

        boxes = (
            results[0]
            .boxes
        )

        detectou = False

        conf = 0

        bbox = [0,0,0,0]

        if len(boxes):

            detectou = True

            box = boxes[0]

            conf = (
                float(
                    box.conf[0]
                )
            )

            x1,y1,x2,y2 = (
                box.xyxy[0]
                .cpu()
                .numpy()
            )

            bbox = [

                x1,
                y1,

                x2-x1,
                y2-y1

            ]

        annotated = (
            results[0]
            .plot(conf=False)
        )

        cv2.imwrite(

            f"{OUTPUT}/{scenario}_{img_name}",

            annotated

        )

        rows.append({

            "image": img_name,

            "scenario": scenario,

            "detected": detectou,

            "confidence": conf,

            "time_ms": elapsed,

            "x": bbox[0],
            "y": bbox[1],
            "w": bbox[2],
            "h": bbox[3]

        })

pd.DataFrame(
    rows
).to_csv(

    "../results/metrics/yolo.csv",

    index=False

)

print("YOLO FINALIZADO")