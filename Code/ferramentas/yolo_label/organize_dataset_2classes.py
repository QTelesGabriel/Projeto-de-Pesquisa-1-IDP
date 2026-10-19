#!/usr/bin/env python3
"""
=============================================================================
Organizador do Dataset YOLOv11n (2 Classes: Armadilha e Alças)
=============================================================================
Divide as fotos rotuladas (labels_2classes) na proporção 75% Treino e 25% Validação.
Copia os arquivos para a estrutura padrão do YOLO:
  yolo_dataset/images/train
  yolo_dataset/images/val
  yolo_dataset/labels/train
  yolo_dataset/labels/val
=============================================================================
"""

import os
import shutil
import argparse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_ORIGEM = os.path.join(SCRIPT_DIR, "dataset")
DEFAULT_DESTINO = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "yolo_dataset"))


def organizar_dataset(origem=DEFAULT_ORIGEM, destino=DEFAULT_DESTINO, labels_dir_name="labels_2classes"):
    pasta_imagens = os.path.join(origem, "imagens")
    pasta_labels = os.path.join(origem, labels_dir_name)

    if not os.path.exists(pasta_imagens):
        print(f"[ERRO] Pasta de imagens não encontrada: {pasta_imagens}")
        return

    if not os.path.exists(pasta_labels):
        print(f"[ERRO] Pasta de rótulos não encontrada: {pasta_labels}")
        return

    # Garante estrutura de destino
    subpastas = [
        "images/train", "images/val",
        "labels/train", "labels/val"
    ]
    for sp in subpastas:
        os.makedirs(os.path.join(destino, sp), exist_ok=True)

    arquivos_img = sorted([
        f for f in os.listdir(pasta_imagens)
        if f.lower().endswith(('.jpg', '.jpeg', '.png'))
    ])

    # Verifica o que já existe no destino
    existentes_train = set(os.listdir(os.path.join(destino, "images", "train")))
    existentes_val = set(os.listdir(os.path.join(destino, "images", "val")))
    todas_existentes = existentes_train.union(existentes_val)

    # Considera apenas imagens que possuem o .txt correspondente
    rotuladas = []
    for img in arquivos_img:
        base = os.path.splitext(img)[0]
        txt = f"{base}.txt"
        if os.path.exists(os.path.join(pasta_labels, txt)):
            rotuladas.append(img)

    novas = [img for img in rotuladas if img not in todas_existentes]

    print("=" * 60)
    print(" ORGANIZADOR DO DATASET YOLOv11n (2 CLASSES)")
    print("=" * 60)
    print(f" Imagens rotuladas encontradas : {len(rotuladas)}")
    print(f" Já presentes no destino       : {len(todas_existentes)}")
    print(f" Novas imagens a organizar     : {len(novas)}")
    print("=" * 60)

    if not novas:
        print("[INFO] Nenhuma nova imagem pendente para cópia. Dataset atualizado!")
        return

    contador_train = 0
    contador_val = 0

    for i, img in enumerate(novas):
        base = os.path.splitext(img)[0]
        txt = f"{base}.txt"

        src_img = os.path.join(pasta_imagens, img)
        src_txt = os.path.join(pasta_labels, txt)

        # Divisão 75/25 (1 em cada 4 vai para validação)
        subpasta = "val" if (i % 4 == 3) else "train"
        if subpasta == "val":
            contador_val += 1
        else:
            contador_train += 1

        dst_img = os.path.join(destino, "images", subpasta, img)
        dst_txt = os.path.join(destino, "labels", subpasta, txt)

        shutil.copy2(src_img, dst_img)
        shutil.copy2(src_txt, dst_txt)

    # Copia data.yaml atualizado para destino se existir
    yaml_origem = os.path.join(SCRIPT_DIR, "data_armadilha_alcas.yaml")
    if os.path.exists(yaml_origem):
        shutil.copy2(yaml_origem, os.path.join(destino, "data.yaml"))
        print("[+] Arquivo data.yaml atualizado copiado com sucesso para o destino.")

    print(f"\n[+] Atualização concluída com sucesso!")
    print(f" -> Novas imagens em Treino    : {contador_train}")
    print(f" -> Novas imagens em Validação : {contador_val}")
    print(f" -> Total no Dataset agora     : {len(todas_existentes) + len(novas)}")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Organiza dataset de 2 classes em train/val.")
    parser.add_argument("--origem", default=DEFAULT_ORIGEM, help="Pasta base de origem contendo imagens e labels_2classes")
    parser.add_argument("--destino", default=DEFAULT_DESTINO, help="Pasta base de destino (yolo_dataset)")
    parser.add_argument("--labels", default="labels_2classes", help="Nome da pasta de labels dentro da origem")
    args = parser.parse_args()

    organizar_dataset(args.origem, args.destino, args.labels)
