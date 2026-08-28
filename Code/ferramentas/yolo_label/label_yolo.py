import cv2
import os
import glob
import sys

IMAGES_DIR = "dataset/imagens"
LABELS_DIR = "dataset/labels"

CLASS_ID = 0

os.makedirs(LABELS_DIR, exist_ok=True)

# ================================
# Coletar e ordenar imagens
# ================================
image_paths = []
for ext in ["*.jpg", "*.png", "*.jpeg"]:
    image_paths.extend(glob.glob(os.path.join(IMAGES_DIR, ext)))

image_paths.sort()

if not image_paths:
    print(f"Nenhuma imagem encontrada em '{IMAGES_DIR}'.")
    sys.exit()

print(f"Total de imagens encontradas: {len(image_paths)}")
print("-" * 50)
print("INSTRUÇÕES DE USO:")
print("- Desenhe a caixa e aperte ENTER ou ESPAÇO.")
print("- Para IMAGENS SEM ARMADILHA (Background): Não desenhe nada, aperte ENTER ou ESPAÇO.")
print("- O programa pulará automaticamente as fotos que já possuem arquivo .txt salvo.")
print("- Feche no 'X' ou use Ctrl+C para parar a qualquer momento.")
print("-" * 50)

# ================================
# Loop de Rotulagem
# ================================
for i, image_path in enumerate(image_paths):
    image_name = os.path.splitext(os.path.basename(image_path))[0]
    label_path = os.path.join(LABELS_DIR, image_name + ".txt")

    # CORREÇÃO CHAVE: Pula a imagem se o label já existir!
    if os.path.exists(label_path):
        print(f"[{i+1}/{len(image_paths)}] Pulando: {os.path.basename(image_path)} (Já rotulado)")
        continue

    image = cv2.imread(image_path)
    if image is None:
        continue

    img_h, img_w = image.shape[:2]
    boxes = []

    print(f"\n[{i+1}/{len(image_paths)}] Rotulando: {os.path.basename(image_path)}")

    while True:
        bbox = cv2.selectROI("Rotulagem YOLO", image, fromCenter=False)
        x, y, w, h = bbox

        # Sair do frame (w=0, h=0)
        if w == 0 or h == 0:
            break

        boxes.append((x, y, w, h))
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # ==========================
    # Salvar YOLO
    # ==========================
    with open(label_path, "w") as f:
        for x, y, w, h in boxes:
            x_center = (x + w/2) / img_w
            y_center = (y + h/2) / img_h
            width = w / img_w
            height = h / img_h

            line = f"{CLASS_ID} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}"
            f.write(line + "\n")

    if not boxes:
        print(f"[SALVO] {label_path} (Background / Vazio)")
    else:
        print(f"[SALVO] {label_path}")

cv2.destroyAllWindows()
print("\nProcesso de rotulagem concluído!")