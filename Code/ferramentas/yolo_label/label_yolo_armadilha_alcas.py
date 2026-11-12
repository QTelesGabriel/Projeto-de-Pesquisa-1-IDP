#!/usr/bin/env python3
"""
=============================================================================
Rotulador Interativo para YOLOv11n - 2 Classes (Armadilha e Alças)
=============================================================================

Classes:
  0: armadilha (a estrutura da armadilha como um todo)
  1: alca (as alças na parte superior da armadilha - normalmente 3 por armadilha)

Recursos:
  - Clique e arraste para desenhar retângulos com retorno visual em tempo real.
  - Comutação inteligente de classes (começa na armadilha e muda para alça).
  - Rastreamento e persistência de progresso em JSON: pare quando quiser e
    continue exatamente de onde parou ao reabrir.
  - Atalho Z para desfazer a última caixa desenhada.
  - Atalho B para voltar à foto anterior e revisar anotações existentes.
  - Atalho R para reiniciar as anotações da foto atual.
  - Atalho V para salvar como background vazio (sem armadilha).
  - Atalho Q ou ESC para salvar o estado e sair com segurança.
=============================================================================
"""

import os
import sys
import glob
import json
import signal
import argparse
from datetime import datetime
import cv2

# =============================================================================
# CONFIGURAÇÕES PADRÃO
# =============================================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_IMAGES_DIR = os.path.join(BASE_DIR, "dataset", "imagens")
# Usamos labels_2classes por padrão para preservar anotações de Pose existentes
DEFAULT_LABELS_DIR = os.path.join(BASE_DIR, "dataset", "labels_2classes")
PROGRESS_FILE = os.path.join(BASE_DIR, "dataset", "progresso_rotulagem_2classes.json")

CLASSES = {
    0: "armadilha",
    1: "alca"
}

# Cores BGR para cada classe
CLASS_COLORS = {
    0: (0, 220, 0),     # Verde para Armadilha
    1: (0, 215, 255)    # Amarelo/Dourado para Alças
}

CLASS_TEXT_COLORS = {
    0: (0, 0, 0),
    1: (0, 0, 0)
}

# =============================================================================
# ESTADO GLOBAL DO MOUSE E DESENHO
# =============================================================================
drawing = False
start_pt = None
current_pt = None
current_class = 0
boxes_current_img = []  # Lista de dicionários: [{'class': id, 'box': (x, y, w, h)}]
request_redraw = True


def mouse_callback(event, x, y, flags, param):
    global drawing, start_pt, current_pt, boxes_current_img, request_redraw

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        start_pt = (x, y)
        current_pt = (x, y)
        request_redraw = True

    elif event == cv2.EVENT_MOUSEMOVE:
        current_pt = (x, y)
        if drawing:
            request_redraw = True

    elif event == cv2.EVENT_LBUTTONUP:
        if drawing and start_pt is not None:
            drawing = False
            x1, y1 = start_pt
            x2, y2 = (x, y)

            xmin = max(0, min(x1, x2))
            ymin = max(0, min(y1, y2))
            xmax = max(0, max(x1, x2))
            ymax = max(0, max(y1, y2))
            w = xmax - xmin
            h = ymax - ymin

            # Ignora cliques acidentais minúsculos (< 4 pixels)
            if w >= 4 and h >= 4:
                global current_class
                boxes_current_img.append({
                    'class': current_class,
                    'box': (xmin, ymin, w, h)
                })

                # Fluxo guiado inteligente:
                # Se acabou de desenhar a armadilha (classe 0), comuta automaticamente para alça (classe 1)
                if current_class == 0:
                    current_class = 1

            start_pt = None
            request_redraw = True


def load_existing_labels(label_path, img_w, img_h):
    """Carrega anotações existentes de um arquivo .txt no formato YOLO."""
    boxes = []
    if not os.path.exists(label_path):
        return boxes

    try:
        with open(label_path, "r", encoding="utf-8") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 5:
                    cls_id = int(parts[0])
                    xc = float(parts[1])
                    yc = float(parts[2])
                    nw = float(parts[3])
                    nh = float(parts[4])

                    w = int(round(nw * img_w))
                    h = int(round(nh * img_h))
                    xmin = int(round((xc - nw / 2.0) * img_w))
                    ymin = int(round((yc - nh / 2.0) * img_h))

                    # Garante limites válidos
                    xmin = max(0, min(img_w - 1, xmin))
                    ymin = max(0, min(img_h - 1, ymin))
                    w = max(1, min(img_w - xmin, w))
                    h = max(1, min(img_h - ymin, h))

                    boxes.append({
                        'class': cls_id,
                        'box': (xmin, ymin, w, h)
                    })
    except Exception as e:
        print(f"[Aviso] Falha ao ler rótulo existente {label_path}: {e}")

    return boxes


def save_yolo_labels(label_path, boxes, img_w, img_h):
    """Salva a lista de bounding boxes no formato YOLO (.txt)."""
    with open(label_path, "w", encoding="utf-8") as f:
        for item in boxes:
            cls_id = item['class']
            x, y, w, h = item['box']

            xc = (x + w / 2.0) / img_w
            yc = (y + h / 2.0) / img_h
            norm_w = w / float(img_w)
            norm_h = h / float(img_h)

            # Limita ao intervalo [0.0, 1.0]
            xc = max(0.0, min(1.0, xc))
            yc = max(0.0, min(1.0, yc))
            norm_w = max(0.0, min(1.0, norm_w))
            norm_h = max(0.0, min(1.0, norm_h))

            line = f"{cls_id} {xc:.6f} {yc:.6f} {norm_w:.6f} {norm_h:.6f}\n"
            f.write(line)


def save_progress(progress_file, last_img_name, last_index, total_images, labeled_count):
    """Registra estado atual no arquivo JSON de progresso."""
    state = {
        "ultima_imagem": last_img_name,
        "ultimo_indice": last_index,
        "total_imagens": total_images,
        "imagens_rotuladas": labeled_count,
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    try:
        os.makedirs(os.path.dirname(progress_file), exist_ok=True)
        with open(progress_file, "w", encoding="utf-8") as f:
            json.dump(state, f, indent=4, ensure_ascii=False)
    except Exception as e:
        print(f"[Aviso] Erro ao salvar arquivo de progresso: {e}")


def load_progress(progress_file):
    """Lê estado prévio do arquivo JSON de progresso se existir."""
    if os.path.exists(progress_file):
        try:
            with open(progress_file, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return None
    return None


def draw_hud(canvas, img_name, img_idx, total_imgs, active_class, boxes):
    """Desenha um painel superior (HUD) com informações visuais e atalhos."""
    h, w = canvas.shape[:2]
    hud_h = 70

    # Fundo semitransparente escuro para o HUD
    overlay = canvas.copy()
    cv2.rectangle(overlay, (0, 0), (w, hud_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.78, canvas, 0.22, 0, canvas)
    cv2.line(canvas, (0, hud_h), (w, hud_h), (80, 80, 80), 1)

    # Contagem de objetos atuais
    count_armadilha = sum(1 for b in boxes if b['class'] == 0)
    count_alcas = sum(1 for b in boxes if b['class'] == 1)

    # Linha 1: Foto atual, classe ativa, contagens
    text_info = f"[{img_idx + 1}/{total_imgs}] {img_name}  |  Armadilhas: {count_armadilha}  |  Alcas: {count_alcas}/3"
    cv2.putText(canvas, text_info, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)

    # Indicador de classe ativa com caixa colorida
    cls_name = CLASSES.get(active_class, "desconhecido").upper()
    cls_color = CLASS_COLORS.get(active_class, (255, 255, 255))
    tag_x = w - 320
    cv2.rectangle(canvas, (tag_x, 8), (w - 15, 34), cls_color, -1)
    tag_text = f"CLASSE ATIVA: [{active_class}] {cls_name}"
    cv2.putText(canvas, tag_text, (tag_x + 10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2, cv2.LINE_AA)

    # Linha 2: Barra de atalhos
    shortcuts = (
        "[ENTER/ESPACO] Salvar & Prox  |  [0/1] Mudar Classe  |  [Z] Desfazer  |  "
        "[R] Limpar Foto  |  [B] Voltar  |  [V] Vazia/Background  |  [Q/ESC] Sair"
    )
    cv2.putText(canvas, shortcuts, (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (180, 220, 255), 1, cv2.LINE_AA)


def render_display(image, boxes, active_class, img_name, img_idx, total_imgs, current_drag=None):
    """Gera o quadro renderizado com caixas, etiquetas e HUD."""
    display = image.copy()
    h, w = display.shape[:2]

    # Contadores para enumerar alças visualmente (#1, #2, #3)
    alca_num = 0

    # Desenha caixas já confirmadas
    for item in boxes:
        cls_id = item['class']
        bx, by, bw, bh = item['box']
        color = CLASS_COLORS.get(cls_id, (0, 255, 0))

        # Retângulo da caixa
        cv2.rectangle(display, (bx, by), (bx + bw, by + bh), color, 2)

        # Monta etiqueta
        if cls_id == 1:
            alca_num += 1
            label = f"[1] alca #{alca_num}"
        else:
            label = f"[0] armadilha"

        # Caixa de fundo para a etiqueta
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        tag_y1 = max(0, by - th - 6)
        tag_y2 = by
        tag_x2 = min(w, bx + tw + 8)
        cv2.rectangle(display, (bx, tag_y1), (tag_x2, tag_y2), color, -1)
        cv2.putText(display, label, (bx + 4, by - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    # Desenha caixa sendo arrastada no momento
    if current_drag is not None:
        (x1, y1), (x2, y2) = current_drag
        xmin = max(0, min(x1, x2))
        ymin = max(0, min(y1, y2))
        xmax = max(0, max(x1, x2))
        ymax = max(0, max(y1, y2))
        color = CLASS_COLORS.get(active_class, (0, 255, 0))

        # Desenha retângulo tracejado/fino em destaque
        cv2.rectangle(display, (xmin, ymin), (xmax, ymax), color, 2)
        cv2.rectangle(display, (xmin - 1, ymin - 1), (xmax + 1, ymax + 1), (255, 255, 255), 1)

    # Mira leve (crosshair) se mouse estiver na tela
    global current_pt
    if current_pt is not None:
        cx, cy = current_pt
        if 0 <= cx < w and 0 <= cy < h:
            cv2.line(display, (cx, 0), (cx, h), (100, 100, 100), 1)
            cv2.line(display, (0, cy), (w, cy), (100, 100, 100), 1)

    # Adiciona o HUD superior
    draw_hud(display, img_name, img_idx, total_imgs, active_class, boxes)

    return display


def main():
    parser = argparse.ArgumentParser(
        description="Rotulador interativo YOLOv11n para 2 classes (Armadilha e Alças) com persistência de progresso."
    )
    parser.add_argument("--images", default=DEFAULT_IMAGES_DIR, help="Caminho para a pasta de imagens")
    parser.add_argument("--labels", default=DEFAULT_LABELS_DIR, help="Caminho para a pasta de salvamento de rótulos .txt")
    parser.add_argument("--start-from", type=int, default=None, help="Índice da foto para iniciar (0-indexado)")
    parser.add_argument("--review", action="store_true", help="Iniciar desde a primeira imagem no modo de revisão")
    args = parser.parse_args()

    images_dir = os.path.abspath(args.images)
    labels_dir = os.path.abspath(args.labels)
    os.makedirs(labels_dir, exist_ok=True)

    # Cria classes.txt na pasta de labels para o YOLO
    classes_file = os.path.join(labels_dir, "classes.txt")
    with open(classes_file, "w", encoding="utf-8") as f:
        f.write("armadilha\nalca\n")

    # Coleta imagens
    image_paths = []
    for ext in ["*.jpg", "*.jpeg", "*.png", "*.JPG", "*.PNG"]:
        image_paths.extend(glob.glob(os.path.join(images_dir, ext)))
    image_paths.sort()

    total_images = len(image_paths)
    if total_images == 0:
        print(f"[ERRO] Nenhuma imagem encontrada em '{images_dir}'.")
        sys.exit(1)

    # Identifica imagens já rotuladas nesta pasta
    labeled_indices = []
    for i, p in enumerate(image_paths):
        base = os.path.splitext(os.path.basename(p))[0]
        lp = os.path.join(labels_dir, base + ".txt")
        if os.path.exists(lp):
            labeled_indices.append(i)

    print("=" * 65)
    print(" ROTULADOR YOLOv11n - 2 CLASSES (ARMADILHA & ALÇAS)")
    print("=" * 65)
    print(f" Imagens encontradas : {total_images}")
    print(f" Já rotuladas        : {len(labeled_indices)} ({(len(labeled_indices)/total_images)*100:.1f}%)")
    print(f" Pasta de Imagens    : {images_dir}")
    print(f" Pasta de Rótulos    : {labels_dir}")
    print("=" * 65)

    # Determina índice inicial
    start_idx = 0
    if args.start_from is not None:
        start_idx = max(0, min(total_images - 1, args.start_from))
        print(f"-> Iniciando no índice solicitado: {start_idx + 1}/{total_images}")
    elif args.review:
        start_idx = 0
        print("-> Modo de revisão ativado: iniciando da primeira imagem.")
    else:
        # Pula as já rotuladas até achar a primeira sem rótulo
        first_unlabeled = None
        for i in range(total_images):
            if i not in labeled_indices:
                first_unlabeled = i
                break

        if first_unlabeled is None:
            print("[INFO] Todas as imagens já foram rotuladas!")
            print("Deseja revisar as imagens a partir do início? (s/N): ", end="")
            try:
                resp = input().strip().lower()
                if resp in ['s', 'sim', 'y', 'yes']:
                    start_idx = 0
                else:
                    print("Encerrando. Nenhuma ação pendente.")
                    return
            except (KeyboardInterrupt, EOFError):
                print("\nOperação cancelada.")
                return
        else:
            start_idx = first_unlabeled
            print(f"-> Retomando automaticamente da imagem {start_idx + 1}/{total_images}: {os.path.basename(image_paths[start_idx])}")

    # Configuração de Janela OpenCV
    window_name = "Rotulador YOLOv11n - Armadilha e Alcas"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    cv2.setMouseCallback(window_name, mouse_callback)

    # Tratamento de interrupção com Ctrl+C
    interrupted = False

    def sigint_handler(sig, frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, sigint_handler)

    idx = start_idx
    global current_class, boxes_current_img, request_redraw, drawing, start_pt, current_pt

    while 0 <= idx < total_images and not interrupted:
        img_path = image_paths[idx]
        img_name = os.path.basename(img_path)
        base_name = os.path.splitext(img_name)[0]
        label_path = os.path.join(labels_dir, base_name + ".txt")

        image = cv2.imread(img_path)
        if image is None:
            print(f"[Aviso] Falha ao ler '{img_path}'. Pulando.")
            idx += 1
            continue

        img_h, img_w = image.shape[:2]

        # Carrega rótulos existentes se houver, caso contrário começa vazio
        boxes_current_img = load_existing_labels(label_path, img_w, img_h)

        # Se já tem armadilha rotulada, sugere classe 1 (alca). Se vazio, classe 0 (armadilha)
        tem_armadilha = any(b['class'] == 0 for b in boxes_current_img)
        current_class = 1 if tem_armadilha else 0

        request_redraw = True
        drawing = False
        start_pt = None
        current_pt = None

        print(f"\n[{idx + 1}/{total_images}] Rotulando: {img_name} ({len(boxes_current_img)} caixas existentes)")

        # Loop de interação para a imagem atual
        while not interrupted:
            # Verifica se a janela foi fechada pelo 'X'
            try:
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    print("\n[PAUSA] Janela fechada pelo usuário.")
                    interrupted = True
                    break
            except Exception:
                pass

            if request_redraw:
                drag_box = (start_pt, current_pt) if (drawing and start_pt and current_pt) else None
                display = render_display(
                    image, boxes_current_img, current_class,
                    img_name, idx, total_images, current_drag=drag_box
                )
                cv2.imshow(window_name, display)
                request_redraw = False

            key = cv2.waitKey(20) & 0xFF

            # ENTER (13) ou ESPAÇO (32): Confirmar e avançar
            if key in [13, 32]:
                save_yolo_labels(label_path, boxes_current_img, img_w, img_h)
                count_arm = sum(1 for b in boxes_current_img if b['class'] == 0)
                count_alc = sum(1 for b in boxes_current_img if b['class'] == 1)
                print(f"[SALVO] {base_name}.txt -> {count_arm} Armadilha(s), {count_alc} Alça(s)")

                # Atualiza progresso persistente
                cur_labeled = len([p for p in image_paths if os.path.exists(os.path.join(labels_dir, os.path.splitext(os.path.basename(p))[0] + ".txt"))])
                save_progress(PROGRESS_FILE, img_name, idx, total_images, cur_labeled)

                idx += 1
                break

            # 0: Selecionar classe Armadilha
            elif key == ord('0'):
                current_class = 0
                request_redraw = True

            # 1: Selecionar classe Alça
            elif key == ord('1'):
                current_class = 1
                request_redraw = True

            # Z ou z: Desfazer última caixa
            elif key in [ord('z'), ord('Z')]:
                if len(boxes_current_img) > 0:
                    removed = boxes_current_img.pop()
                    print(f" -> Desfeita: [{removed['class']}] {CLASSES.get(removed['class'])}")
                    # Se desfez e ficou sem armadilha, volta classe ativa para 0
                    if not any(b['class'] == 0 for b in boxes_current_img):
                        current_class = 0
                    request_redraw = True

            # R ou r: Reiniciar todas as anotações desta foto
            elif key in [ord('r'), ord('R')]:
                boxes_current_img.clear()
                current_class = 0
                print(f" -> Anotações de {img_name} limpas.")
                request_redraw = True

            # B ou b: Voltar para a foto anterior
            elif key in [ord('b'), ord('B')]:
                if idx > 0:
                    print(" -> Voltando para a imagem anterior...")
                    idx -= 1
                    break
                else:
                    print(" -> Já está na primeira imagem!")

            # V ou v: Salvar imagem como VAZIA / BACKGROUND
            elif key in [ord('v'), ord('V')]:
                boxes_current_img.clear()
                save_yolo_labels(label_path, [], img_w, img_h)
                print(f"[SALVO] {base_name}.txt marcado como Background (vazio).")
                cur_labeled = len([p for p in image_paths if os.path.exists(os.path.join(labels_dir, os.path.splitext(os.path.basename(p))[0] + ".txt"))])
                save_progress(PROGRESS_FILE, img_name, idx, total_images, cur_labeled)
                idx += 1
                break

            # Q, q ou ESC (27): Pausar e sair com segurança
            elif key in [ord('q'), ord('Q'), 27]:
                print("\n[PAUSA] Comando de saída acionado pelo usuário.")
                interrupted = True
                break

    cv2.destroyAllWindows()

    # Mensagem final e resumo de progresso
    cur_labeled = len([p for p in image_paths if os.path.exists(os.path.join(labels_dir, os.path.splitext(os.path.basename(p))[0] + ".txt"))])
    last_processed = image_paths[idx - 1] if idx > 0 else (image_paths[0] if total_images > 0 else "")
    last_name = os.path.basename(last_processed) if last_processed else "Nenhuma"

    save_progress(PROGRESS_FILE, last_name, max(0, idx - 1), total_images, cur_labeled)

    print("\n" + "=" * 65)
    print(" SESSÃO FINALIZADA")
    print("=" * 65)
    print(f" Total de imagens no dataset : {total_images}")
    print(f" Imagens rotuladas salvas    : {cur_labeled} ({(cur_labeled/total_images)*100:.1f}%)")
    print(f" Imagens restantes           : {total_images - cur_labeled}")
    if cur_labeled < total_images:
        print(f" Progresso salvo em          : {PROGRESS_FILE}")
        print(" Para continuar de onde parou, basta executar o script novamente:")
        print(f"   python3 {os.path.basename(__file__)}")
    else:
        print(" PARABÉNS! Todas as fotos do dataset foram concluídas!")
    print("=" * 65)


if __name__ == "__main__":
    main()
