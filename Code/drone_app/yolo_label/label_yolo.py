import cv2
import os
import glob

IMAGES_DIR = "dataset/imagens"
LABELS_DIR = "dataset/labels"

CLASS_ID = 0

os.makedirs(LABELS_DIR, exist_ok=True)

# ================================

image_paths = []

for ext in ["*.jpg", "*.png", "*.jpeg"]:
    image_paths.extend(
        glob.glob(os.path.join(IMAGES_DIR, ext))
    )

image_paths.sort()

# ================================

for image_path in image_paths:

    image = cv2.imread(image_path)

    if image is None:
        continue

    img_h, img_w = image.shape[:2]

    boxes = []

    while True:

        bbox = cv2.selectROI(
            "Selecione",
            image,
            fromCenter=False
        )

        x, y, w, h = bbox

        # ESC / cancelar
        if w == 0 or h == 0:
            break

        boxes.append((x, y, w, h))

        # Desenha direto na imagem
        cv2.rectangle(
            image,
            (x, y),
            (x + w, y + h),
            (0, 255, 0),
            2
        )

    # ==========================
    # Salvar YOLO
    # ==========================

    image_name = os.path.splitext(
        os.path.basename(image_path)
    )[0]

    label_path = os.path.join(
        LABELS_DIR,
        image_name + ".txt"
    )

    with open(label_path, "w") as f:

        for x, y, w, h in boxes:

            x_center = (x + w/2) / img_w
            y_center = (y + h/2) / img_h

            width = w / img_w
            height = h / img_h

            line = (
                f"{CLASS_ID} "
                f"{x_center:.6f} "
                f"{y_center:.6f} "
                f"{width:.6f} "
                f"{height:.6f}"
            )

            f.write(line + "\n")

    print(f"[SALVO] {label_path}")

cv2.destroyAllWindows()