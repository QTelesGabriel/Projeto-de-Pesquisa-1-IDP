#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Benchmark Comparativo dos Modelos YOLO para a 3ª Etapa (Etapa Final - Alinhamento a 0,20 m).
Compara as 4 arquiteturas treinadas no Dataset Etapa Final (1 classe: 'Alinhamento'):
  - YOLOv8n  (YOLOv8n_EtapaFinal_Alinhamento)
  - YOLOv10n (YOLOv10n_EtapaFinal_Alinhamento)
  - YOLOv11n (YOLOv11n_EtapaFinal_Alinhamento-2)
  - YOLO26n  (YOLO26n_EtapaFinal_Alinhamento)

Avalia:
  1. Métricas padrão de detecção (P, R, mAP@50, mAP@50-95) em 'val' (80 imgs) e 'test' (40 imgs).
  2. Métricas geométricas de desempate para servovisão a 0,20 m (conf=0.5, batch=1):
     - IoU médio contínuo (sem teto artificial de 99,50% do mAP)
     - Erro radial do centroide (média, P95 e máximo) em pixels e em mm @ 0,20 m
     - Viés sistemático do centroide (dx, dy) em mm @ 0,20 m
     - Latência real batch=1 (ms) e taxa de quadros (FPS)
  3. Modo --rapido: extrai e analisa o histórico de treinamento (results.csv) sem PyTorch.
"""

import argparse
import csv
import glob
import math
import os
import statistics
import time

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
MODELOS_DIR = os.path.join(ROOT_DIR, 'Code', 'modelos_IA')
DATASET_DIR = os.path.join(ROOT_DIR, 'Code', 'ferramentas', 'datasets', 'Dataset Etapa Final')
DATASET_YAML = os.path.join(DATASET_DIR, 'data.yaml')
RESULTADOS_DIR = os.path.join(os.path.dirname(__file__), 'resultados')
os.makedirs(RESULTADOS_DIR, exist_ok=True)

# Parâmetros ópticos da câmera SIYI A8 mini usados em rastreador_fino.py @ altitude bruta 0,20 m
FX_PX = 1124.0
FY_PX = 1124.0
ALTITUDE_BRUTA_REF_M = 0.20
ALTURA_CAMERA_OFFSET_M = 0.320
ALTURA_GAIOLA_M = 0.317
ALTITUDE_RELATIVA_REF_M = (ALTITUDE_BRUTA_REF_M + ALTURA_CAMERA_OFFSET_M) - ALTURA_GAIOLA_M  # 0,203 m
MM_POR_PX_X = (ALTITUDE_RELATIVA_REF_M / FX_PX) * 1000.0  # ~0,1806 mm/px
MM_POR_PX_Y = (ALTITUDE_RELATIVA_REF_M / FY_PX) * 1000.0

MODELOS_ETAPA3 = [
    (
        'YOLOv8n',
        os.path.join(MODELOS_DIR, 'YOLOv8n_EtapaFinal_Alinhamento', 'weights', 'best.pt'),
        os.path.join(MODELOS_DIR, 'YOLOv8n_EtapaFinal_Alinhamento', 'results.csv'),
    ),
    (
        'YOLOv10n',
        os.path.join(MODELOS_DIR, 'YOLOv10n_EtapaFinal_Alinhamento', 'weights', 'best.pt'),
        os.path.join(MODELOS_DIR, 'YOLOv10n_EtapaFinal_Alinhamento', 'results.csv'),
    ),
    (
        'YOLOv11n',
        os.path.join(MODELOS_DIR, 'YOLOv11n_EtapaFinal_Alinhamento-2', 'weights', 'best.pt'),
        os.path.join(MODELOS_DIR, 'YOLOv11n_EtapaFinal_Alinhamento-2', 'results.csv'),
    ),
    (
        'YOLO26n',
        os.path.join(MODELOS_DIR, 'YOLO26n_EtapaFinal_Alinhamento', 'weights', 'best.pt'),
        os.path.join(MODELOS_DIR, 'YOLO26n_EtapaFinal_Alinhamento', 'results.csv'),
    ),
]


def extrair_metricas_treino(caminho_csv):
    """Extrai o melhor checkpoint segundo o fitness da Ultralytics (0.1*mAP50 + 0.9*mAP50-95) e estabilidade."""
    if not os.path.exists(caminho_csv):
        return None
    with open(caminho_csv, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        headers = [h.strip() for h in next(reader)]
        rows = [row for row in reader if row]

    idx = {name: i for i, name in enumerate(headers)}
    best_fit = -1.0
    best_row = None
    m95_post100 = []
    vbox_post100 = []
    epochs_teto = 0

    for i, row in enumerate(rows):
        m50 = float(row[idx['metrics/mAP50(B)']].strip())
        m95 = float(row[idx['metrics/mAP50-95(B)']].strip())
        vbox = float(row[idx['val/box_loss']].strip())
        fit = 0.1 * m50 + 0.9 * m95
        if fit > best_fit:
            best_fit = fit
            best_row = row
        if m95 >= 0.995:
            epochs_teto += 1
        if i >= 100:
            m95_post100.append(m95 * 100.0)
            vbox_post100.append(vbox)

    if best_row is None:
        return None

    return {
        'epocas_totais': len(rows),
        'melhor_epoca': int(float(best_row[idx['epoch']].strip())),
        'P': float(best_row[idx['metrics/precision(B)']].strip()) * 100.0,
        'R': float(best_row[idx['metrics/recall(B)']].strip()) * 100.0,
        'mAP50': float(best_row[idx['metrics/mAP50(B)']].strip()) * 100.0,
        'mAP50_95': float(best_row[idx['metrics/mAP50-95(B)']].strip()) * 100.0,
        'val_box_loss_best': float(best_row[idx['val/box_loss']].strip()),
        'val_box_loss_min': min(float(r[idx['val/box_loss']].strip()) for r in rows),
        'm95_post100_media': statistics.mean(m95_post100) if m95_post100 else None,
        'vbox_post100_media': statistics.mean(vbox_post100) if vbox_post100 else None,
        'epocas_teto_995': epochs_teto,
    }


def _carregar_ground_truth(split_nome):
    """Lê pares (caminho_imagem, (cx_norm, cy_norm, w_norm, h_norm)) do split especificado."""
    pasta_split = 'valid' if split_nome == 'val' else split_nome
    imgs = sorted(glob.glob(os.path.join(DATASET_DIR, pasta_split, 'images', '*.jpg')))
    pares = []
    for img_path in imgs:
        stem = os.path.splitext(os.path.basename(img_path))[0]
        lbl_path = os.path.join(DATASET_DIR, pasta_split, 'labels', f'{stem}.txt')
        if not os.path.exists(lbl_path):
            continue
        with open(lbl_path, 'r', encoding='utf-8') as f:
            linhas = [l.strip() for l in f if l.strip()]
        if not linhas:
            continue
        partes = linhas[0].split()
        _, cx_n, cy_n, w_n, h_n = map(float, partes[:5])
        pares.append((img_path, (cx_n, cy_n, w_n, h_n)))
    return pares


def _calcular_iou_xyxy(box_a, box_b):
    ix1 = max(box_a[0], box_b[0])
    iy1 = max(box_a[1], box_b[1])
    ix2 = min(box_a[2], box_b[2])
    iy2 = min(box_a[3], box_b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, box_a[2] - box_a[0]) * max(0.0, box_a[3] - box_a[1])
    area_b = max(0.0, box_b[2] - box_b[0]) * max(0.0, box_b[3] - box_b[1])
    uniao = area_a + area_b - inter
    return inter / uniao if uniao > 0.0 else 0.0


def _percentil(valores, p):
    if not valores:
        return 0.0
    ordenados = sorted(valores)
    k = (len(ordenados) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return ordenados[int(k)]
    return ordenados[f] * (c - k) + ordenados[c] * (k - f)


def avaliar_geometria_e_fps_ao_vivo(modelo, splits=('val', 'test')):
    """
    Executa inferência batch=1 (idêntica ao rastreador_fino.py: conf=0.5, imgsz=640)
    para medir latência real, FPS, IoU contínuo e erro de centroide em px e mm @ 0,20 m.
    """
    import cv2

    amostras = []
    for sp in splits:
        for img_path, gt_norm in _carregar_ground_truth(sp):
            img_bgr = cv2.imread(img_path)
            if img_bgr is not None:
                amostras.append((sp, img_bgr, gt_norm))

    if not amostras:
        return None

    # Aquecimento da GPU/CPU antes de medir latência
    print(f"    -> Aquecendo e avaliando geometria/FPS batch=1 ({len(amostras)} imagens)...", flush=True)
    for _, img_bgr, _ in amostras[:min(5, len(amostras))]:
        modelo.predict(img_bgr, verbose=False, conf=0.5, imgsz=640)

    erros_rad_px, erros_rad_mm = [], []
    dx_mm_list, dy_mm_list = [], []
    ious, tempos_ms = [], []
    erros_test_mm, ious_test = [], []
    detectados = 0

    for idx_img, (sp, img_bgr, (gt_cx_n, gt_cy_n, gt_w_n, gt_h_n)) in enumerate(amostras, 1):
        if idx_img % 40 == 0 or idx_img == len(amostras):
            print(f"       progresso: {idx_img}/{len(amostras)} imagens...", flush=True)
        h_img, w_img = img_bgr.shape[:2]
        gt_cx = gt_cx_n * w_img
        gt_cy = gt_cy_n * h_img
        gt_bw = gt_w_n * w_img
        gt_bh = gt_h_n * h_img
        gt_xyxy = (gt_cx - gt_bw / 2.0, gt_cy - gt_bh / 2.0, gt_cx + gt_bw / 2.0, gt_cy + gt_bh / 2.0)

        t0 = time.perf_counter()
        res = modelo.predict(img_bgr, verbose=False, conf=0.5, imgsz=640)
        t1 = time.perf_counter()
        tempos_ms.append((t1 - t0) * 1000.0)

        boxes = res[0].boxes
        if boxes is None or len(boxes) == 0:
            continue

        detectados += 1
        best = max(boxes, key=lambda b: float(b.conf[0].item()))
        x1, y1, x2, y2 = map(float, best.xyxy[0].cpu().numpy())
        pred_cx = (x1 + x2) / 2.0
        pred_cy = (y1 + y2) / 2.0

        dx_px = pred_cx - gt_cx
        dy_px = pred_cy - gt_cy
        err_px = math.hypot(dx_px, dy_px)
        dx_mm = dx_px * MM_POR_PX_X
        dy_mm = dy_px * MM_POR_PX_Y
        err_mm = math.hypot(dx_mm, dy_mm)
        iou_val = _calcular_iou_xyxy((x1, y1, x2, y2), gt_xyxy)

        erros_rad_px.append(err_px)
        erros_rad_mm.append(err_mm)
        dx_mm_list.append(dx_mm)
        dy_mm_list.append(dy_mm)
        ious.append(iou_val)
        if sp == 'test':
            erros_test_mm.append(err_mm)
            ious_test.append(iou_val)

    lat_media_ms = statistics.mean(tempos_ms)
    return {
        'total_imgs': len(amostras),
        'taxa_deteccao_pct': (detectados / len(amostras)) * 100.0,
        'latencia_b1_ms': lat_media_ms,
        'fps_b1': 1000.0 / lat_media_ms if lat_media_ms > 0 else 0.0,
        'iou_medio_pct': statistics.mean(ious) * 100.0 if ious else 0.0,
        'test_iou_medio_pct': statistics.mean(ious_test) * 100.0 if ious_test else None,
        'erro_centro_px_medio': statistics.mean(erros_rad_px) if erros_rad_px else None,
        'erro_centro_mm_medio': statistics.mean(erros_rad_mm) if erros_rad_mm else None,
        'test_erro_centro_mm_medio': statistics.mean(erros_test_mm) if erros_test_mm else None,
        'erro_centro_mm_p95': _percentil(erros_rad_mm, 95) if erros_rad_mm else None,
        'erro_centro_mm_max': max(erros_rad_mm) if erros_rad_mm else None,
        'vies_dx_mm': statistics.mean(dx_mm_list) if dx_mm_list else None,
        'vies_dy_mm': statistics.mean(dy_mm_list) if dy_mm_list else None,
    }


def avaliar_modelo_ao_vivo(peso_path):
    """Avalia um modelo no split 'val', no split 'test' e na geometria/FPS batch=1."""
    from ultralytics import YOLO

    modelo = YOLO(peso_path)
    metricas_splits = {}

    for sp in ('val', 'test'):
        print(f"    -> Executando model.val(split='{sp}')...", flush=True)
        m = modelo.val(data=DATASET_YAML, split=sp, verbose=False, imgsz=640, batch=8, plots=False)
        p = float(m.box.mp) * 100.0 if hasattr(m.box, 'mp') else float(m.box.p.mean()) * 100.0
        r = float(m.box.mr) * 100.0 if hasattr(m.box, 'mr') else float(m.box.r.mean()) * 100.0
        spd = m.speed
        lat = spd.get('preprocess', 0.0) + spd.get('inference', 0.0) + spd.get('postprocess', 0.0)
        metricas_splits[sp] = {
            'P': p,
            'R': r,
            'mAP50': float(m.box.map50) * 100.0,
            'mAP50_95': float(m.box.map) * 100.0,
            'latencia_val_ms': lat,
            'fps_val': 1000.0 / lat if lat > 0 else 0.0,
        }

    geo = avaliar_geometria_e_fps_ao_vivo(modelo, splits=('val', 'test'))
    return metricas_splits, geo


def gerar_tabela_latex(registros):
    """Gera tabela LaTeX consolidada para a 3ª Etapa (Alinhamento a 0,20 m)."""
    linhas = [
        "% ==================== TABELA LATEX: 3ª ETAPA (ALINHAMENTO @ 0,20m) ====================",
        "\\begin{table}[H]",
        "    \\centering",
        "    \\small",
        "    \\setlength{\\tabcolsep}{4.5pt}",
        "    \\captionsetup{justification=centering}",
        "    \\caption{Comparativo de desempenho entre arquiteturas YOLO na 3ª Etapa (alinhamento fino a 0,20 m)}",
        "    \\label{tab:comparativo_yolo_etapa3_alinhamento}",
        "    \\vspace{0.2cm}",
        "    \\begin{tabular}{lccccccc}",
        "        \\hline",
        "        \\textbf{Modelo} & \\textbf{$mAP@50$ (Val)} & \\textbf{$mAP@50\\text{--}95$ (Val)} & \\textbf{$mAP@50\\text{--}95$ (Test)} & \\textbf{IoU Médio} & \\textbf{Erro Centro (mm)} & \\textbf{Lat. (ms)} & \\textbf{FPS} \\\\",
        "        \\hline",
    ]
    for r in registros:
        m50_v = f"{r['Val_mAP50_pct']:.2f}\\%"
        m95_v = f"{r['Val_mAP50_95_pct']:.2f}\\%"
        m95_t = f"{r['Test_mAP50_95_pct']:.2f}\\%" if r['Test_mAP50_95_pct'] is not None else "--"
        iou_s = f"{r['IoU_Medio_pct']:.2f}\\%" if r['IoU_Medio_pct'] is not None else "--"
        err_s = f"{r['Erro_Centro_mm_Medio']:.2f}" if r['Erro_Centro_mm_Medio'] is not None else "--"
        lat_s = f"{r['Latencia_B1_ms']:.1f}" if r['Latencia_B1_ms'] is not None else "--"
        fps_s = f"{r['FPS_B1']:.1f}" if r['FPS_B1'] is not None else "--"
        linhas.append(
            f"        {r['Arquitetura']:<9} & {m50_v:<10} & {m95_v:<10} & {m95_t:<10} & {iou_s:<10} & {err_s:<10} & {lat_s:<8} & {fps_s} \\\\"
        )
    linhas.extend([
        "        \\hline",
        "    \\end{tabular}",
        "    \\vspace{0.2cm}",
        "    \\begin{flushleft}",
        "        \\footnotesize Fonte: elaboração própria.",
        "    \\end{flushleft}",
        "\\end{table}",
    ])
    return "\n".join(linhas)


def main():
    parser = argparse.ArgumentParser(description="Benchmark da 3ª Etapa (Alinhamento @ 0.20 m)")
    parser.add_argument('--rapido', action='store_true', help="Analisa apenas os results.csv de treino sem carregar PyTorch/YOLO.")
    args = parser.parse_args()

    print("\n" + "=" * 95)
    print("🎯 BENCHMARK DA 3ª ETAPA: ALINHAMENTO FINO A 0,20 m (Dataset Etapa Final)")
    print(f"   Dataset: {DATASET_YAML}")
    print(f"   Escala ótica @ 0,20 m (alt_rel={ALTURA_CAMERA_OFFSET_M - ALTURA_GAIOLA_M + ALTITUDE_BRUTA_REF_M:.3f}m): {MM_POR_PX_X:.4f} mm/px")
    print("=" * 95)

    registros = []
    for nome, peso_path, csv_path in MODELOS_ETAPA3:
        print(f"\n[*] Avaliando {nome}...")
        if not os.path.exists(peso_path):
            print(f"    [!] Pesos não encontrados: {peso_path}")
            continue

        treino = extrair_metricas_treino(csv_path)
        splits, geo = None, None
        if not args.rapido:
            try:
                splits, geo = avaliar_modelo_ao_vivo(peso_path)
            except Exception as e:
                print(f"    [!] Falha na avaliação ao vivo ({e}). Usando dados de results.csv...")

        reg = {
            'Arquitetura': nome,
            'Epocas': treino['epocas_totais'] if treino else None,
            'Melhor_Epoca': treino['melhor_epoca'] if treino else None,
            'Val_P_pct': round(splits['val']['P'], 2) if splits else round(treino['P'], 2),
            'Val_R_pct': round(splits['val']['R'], 2) if splits else round(treino['R'], 2),
            'Val_mAP50_pct': round(splits['val']['mAP50'], 2) if splits else round(treino['mAP50'], 2),
            'Val_mAP50_95_pct': round(splits['val']['mAP50_95'], 2) if splits else round(treino['mAP50_95'], 2),
            'Val_Box_Loss_Best': round(treino['val_box_loss_best'], 4) if treino else None,
            'Val_Box_Loss_Min': round(treino['val_box_loss_min'], 4) if treino else None,
            'Epocas_mAP95_995': treino['epocas_teto_995'] if treino else None,
            'Test_P_pct': round(splits['test']['P'], 2) if splits else None,
            'Test_R_pct': round(splits['test']['R'], 2) if splits else None,
            'Test_mAP50_pct': round(splits['test']['mAP50'], 2) if splits else None,
            'Test_mAP50_95_pct': round(splits['test']['mAP50_95'], 2) if splits else None,
            'IoU_Medio_pct': round(geo['iou_medio_pct'], 2) if geo else None,
            'Test_IoU_Medio_pct': round(geo['test_iou_medio_pct'], 2) if geo and geo['test_iou_medio_pct'] is not None else None,
            'Erro_Centro_px_Medio': round(geo['erro_centro_px_medio'], 2) if geo and geo['erro_centro_px_medio'] is not None else None,
            'Erro_Centro_mm_Medio': round(geo['erro_centro_mm_medio'], 3) if geo and geo['erro_centro_mm_medio'] is not None else None,
            'Test_Erro_Centro_mm_Medio': round(geo['test_erro_centro_mm_medio'], 3) if geo and geo['test_erro_centro_mm_medio'] is not None else None,
            'Erro_Centro_mm_P95': round(geo['erro_centro_mm_p95'], 3) if geo and geo['erro_centro_mm_p95'] is not None else None,
            'Erro_Centro_mm_Max': round(geo['erro_centro_mm_max'], 3) if geo and geo['erro_centro_mm_max'] is not None else None,
            'Vies_dX_mm': round(geo['vies_dx_mm'], 3) if geo and geo['vies_dx_mm'] is not None else None,
            'Vies_dY_mm': round(geo['vies_dy_mm'], 3) if geo and geo['vies_dy_mm'] is not None else None,
            'Taxa_Deteccao_pct': round(geo['taxa_deteccao_pct'], 1) if geo else None,
            'Latencia_B1_ms': round(geo['latencia_b1_ms'], 2) if geo else None,
            'FPS_B1': round(geo['fps_b1'], 1) if geo else None,
            'Caminho_Pesos': peso_path,
        }
        registros.append(reg)

    if not registros:
        print("\n[!] Nenhum resultado obtido.")
        return

    csv_out = os.path.join(RESULTADOS_DIR, 'benchmark_etapa3_alinhamento.csv')
    with open(csv_out, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(registros[0].keys()))
        writer.writeheader()
        writer.writerows(registros)

    tex_out = os.path.join(RESULTADOS_DIR, 'tabela_etapa3_alinhamento.tex')
    tex_str = gerar_tabela_latex(registros)
    with open(tex_out, 'w', encoding='utf-8') as f:
        f.write(tex_str)

    print("\n" + "=" * 95)
    print("📋 RESUMO COMPARATIVO - 3ª ETAPA (ALINHAMENTO @ 0,20 m)")
    print("=" * 95)
    fmt = "{:<9} {:<8} {:<9} {:<9} {:<9} {:<9} {:<11} {:<10} {:<8}"
    print(fmt.format("Modelo", "Val95%", "Test95%", "IoU(%)", "Erro(mm)", "P95(mm)", "Vies(X,Y)", "BoxLoss", "FPS(b1)"))
    print("-" * 95)
    for r in registros:
        t95 = f"{r['Test_mAP50_95_pct']:.2f}" if r['Test_mAP50_95_pct'] is not None else "--"
        iou = f"{r['IoU_Medio_pct']:.2f}" if r['IoU_Medio_pct'] is not None else "--"
        emm = f"{r['Erro_Centro_mm_Medio']:.3f}" if r['Erro_Centro_mm_Medio'] is not None else "--"
        p95 = f"{r['Erro_Centro_mm_P95']:.3f}" if r['Erro_Centro_mm_P95'] is not None else "--"
        vies = f"{r['Vies_dX_mm']:+.2f},{r['Vies_dY_mm']:+.2f}" if r['Vies_dX_mm'] is not None else "--"
        bl = f"{r['Val_Box_Loss_Best']:.4f}" if r['Val_Box_Loss_Best'] is not None else "--"
        fps = f"{r['FPS_B1']:.1f}" if r['FPS_B1'] is not None else "--"
        print(fmt.format(r['Arquitetura'], f"{r['Val_mAP50_95_pct']:.2f}", t95, iou, emm, p95, vies, bl, fps))
    print("=" * 95)

    # Ranking automático se modo ao vivo rodou; caso contrário explica o empate no modo --rapido
    com_geo = [r for r in registros if r['Erro_Centro_mm_Medio'] is not None]
    if com_geo:
        # Critério principal para 0,20 m: menor erro médio de centroide (desde que Test mAP50-95 >= 95% e FPS >= 20)
        elegiveis = [r for r in com_geo if (r['FPS_B1'] or 0) >= 20.0 and (r['Taxa_Deteccao_pct'] or 0) >= 99.0]
        pool = elegiveis if elegiveis else com_geo
        vencedor = min(pool, key=lambda r: (r['Erro_Centro_mm_Medio'], -r['IoU_Medio_pct'], -r['FPS_B1']))
        print(f"\n🏆 MELHOR MODELO PARA 3ª ETAPA (0,20 m): {vencedor['Arquitetura']}")
        print(
            f"   Erro médio de centroide = {vencedor['Erro_Centro_mm_Medio']:.3f} mm "
            f"(Test: {vencedor['Test_Erro_Centro_mm_Medio']:.3f} mm | P95 = {vencedor['Erro_Centro_mm_P95']:.3f} mm) | "
            f"IoU = {vencedor['IoU_Medio_pct']:.2f}% (Test: {vencedor['Test_IoU_Medio_pct']:.2f}%) | "
            f"FPS = {vencedor['FPS_B1']:.1f}"
        )
    else:
        print("\n[i] Avaliação ao vivo não executada (sem métricas do split 'test', IoU contínuo, erro de centroide e FPS).")
        print("    Nos CSVs de treino (valid), YOLOv10n, YOLOv11n e YOLO26n empataram no teto de 99.50% mAP@50-95.")
        print("    Execute sem '--rapido' no ambiente com PyTorch/Ultralytics para o desempate completo.")

    print(f"\n[+] CSV salvo em  : {csv_out}")
    print(f"[+] LaTeX salvo em: {tex_out}\n")


if __name__ == "__main__":
    main()
