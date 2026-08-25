import os
import shutil
from pathlib import Path

# ============================================================
# CONFIGURE AQUI
# ============================================================

# Pasta onde estão TODOS os labels originais
SOURCE_LABELS = r"/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/drone_app/yolo_label/dataset/labels"

# Pasta raiz do dataset YOLO
DATASET_PATH = r"/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/drone_app/vision_yolo/dataset"

# Estrutura esperada:
#
# dataset/
# ├── images/
# │   ├── train/
# │   └── val/
# └── labels/
#     ├── train/
#     └── val/
#
# ============================================================


def garantir_pastas():
    """
    Cria as pastas labels/train e labels/val se não existirem.
    """
    os.makedirs(os.path.join(DATASET_PATH, "labels", "train"), exist_ok=True)
    os.makedirs(os.path.join(DATASET_PATH, "labels", "val"), exist_ok=True)


def copiar_labels(images_folder, labels_destino):
    """
    Para cada imagem da pasta images_folder:
    - procura o label correspondente
    - copia para labels_destino
    """

    imagens = Path(images_folder)

    total = 0
    encontrados = 0
    faltando = []

    for imagem in imagens.iterdir():

        # Ignora arquivos que não sejam imagem
        if imagem.suffix.lower() not in [".jpg", ".jpeg", ".png"]:
            continue

        total += 1

        # Nome do txt correspondente
        nome_label = imagem.stem + ".txt"

        origem_label = os.path.join(SOURCE_LABELS, nome_label)

        if os.path.exists(origem_label):

            destino = os.path.join(labels_destino, nome_label)

            shutil.copy2(origem_label, destino)

            encontrados += 1

            print(f"[OK] {nome_label}")

        else:
            faltando.append(nome_label)
            print(f"[FALTANDO] {nome_label}")

    print("\n===================================")
    print(f"Pasta analisada: {images_folder}")
    print(f"Total de imagens: {total}")
    print(f"Labels encontrados: {encontrados}")
    print(f"Labels faltando: {len(faltando)}")

    if faltando:
        print("\nArquivos faltando:")
        for f in faltando:
            print(f" - {f}")

    print("===================================\n")


def main():

    garantir_pastas()

    # Caminhos das imagens
    train_images = os.path.join(DATASET_PATH, "images", "train")
    val_images = os.path.join(DATASET_PATH, "images", "val")

    # Caminhos destino dos labels
    train_labels = os.path.join(DATASET_PATH, "labels", "train")
    val_labels = os.path.join(DATASET_PATH, "labels", "val")

    print("\n========== TRAIN ==========\n")
    copiar_labels(train_images, train_labels)

    print("\n========== VAL ==========\n")
    copiar_labels(val_images, val_labels)

    print("\nProcesso finalizado.")


if __name__ == "__main__":
    main()