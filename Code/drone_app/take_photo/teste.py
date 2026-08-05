import cv2
import os
import math


# ==========================
# CONFIGURAÇÕES
# ==========================

INPUT_IMAGE = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/drone_app/take_photo/photos/foto_1782325134.png"
OUTPUT_IMAGE = "erro_visual.png"

# Faixas HSV para vermelho
LOWER_RED1 = (0, 80, 50)
UPPER_RED1 = (10, 255, 255)

LOWER_RED2 = (170, 80, 50)
UPPER_RED2 = (180, 255, 255)

# ==========================
# CARREGAR
# ==========================

frame = cv2.imread(INPUT_IMAGE)

if frame is None:
    raise Exception("Imagem não encontrada")

output = frame.copy()

height, width = frame.shape[:2]

# Centro da câmera
cx_cam = width // 2
cy_cam = height // 2

# ==========================
# HSV
# ==========================

hsv = cv2.cvtColor(
    frame,
    cv2.COLOR_BGR2HSV
)

mask1 = cv2.inRange(
    hsv,
    LOWER_RED1,
    UPPER_RED1
)

mask2 = cv2.inRange(
    hsv,
    LOWER_RED2,
    UPPER_RED2
)

mask = cv2.bitwise_or(
    mask1,
    mask2
)

kernel = cv2.getStructuringElement(
    cv2.MORPH_RECT,
    (5, 5)
)

mask = cv2.morphologyEx(
    mask,
    cv2.MORPH_OPEN,
    kernel
)

mask = cv2.morphologyEx(
    mask,
    cv2.MORPH_CLOSE,
    kernel
)

# ==========================
# CONTORNOS
# ==========================

contours, _ = cv2.findContours(
    mask,
    cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE
)

# Centro da câmera
cv2.circle(
    output,
    (cx_cam, cy_cam),
    8,
    (255, 0, 0),
    -1
)

cv2.putText(
    output,
    "Camera",
    (cx_cam + 10, cy_cam - 10),
    cv2.FONT_HERSHEY_SIMPLEX,
    0.7,
    (255, 0, 0),
    2
)

if contours:

    c = max(
        contours,
        key=cv2.contourArea
    )

    x, y, w, h = cv2.boundingRect(c)

    # Bounding box
    cv2.rectangle(
        output,
        (x, y),
        (x + w, y + h),
        (0, 255, 0),
        3
    )

    # Centro do objeto
    cx_obj = x + w // 2
    cy_obj = y + h // 2

    cv2.circle(
        output,
        (cx_obj, cy_obj),
        8,
        (0, 255, 0),
        -1
    )

    # Linha do erro
    cv2.line(
        output,
        (cx_cam, cy_cam),
        (cx_obj, cy_obj),
        (0, 0, 255),
        3
    )

    # Erro
    erro_x = cx_obj - cx_cam
    erro_y = cy_obj - cy_cam

    distancia = math.sqrt(
        erro_x**2 +
        erro_y**2
    )

    texto = (
        f"Erro X: {erro_x}px | "
        f"Erro Y: {erro_y}px | "
        f"Dist: {distancia:.1f}px"
    )

    cv2.putText(
        output,
        texto,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

else:

    cv2.putText(
        output,
        "Objeto vermelho nao encontrado",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 255),
        2
    )

# ==========================
# SALVAR
# ==========================

cv2.imwrite(
    OUTPUT_IMAGE,
    output
)

print("Imagem salva:", OUTPUT_IMAGE)