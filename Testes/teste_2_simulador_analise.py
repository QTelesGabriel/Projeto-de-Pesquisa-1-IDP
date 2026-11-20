#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analisador Científico do Teste 2 no Simulador Gazebo (EKF + Oclusão Real).

Lê exclusivamente os arquivos gerados em `Testes/resultados/voos_ekf_simulador/`
(sem ler nem alterar `relatorio_testes_monte_carlo.csv` ou `Code/missao/telemetria/`)
e gera:
  - `Testes/resultados/teste2_simulador_resumo.csv`
  - `Testes/graficos/teste2_ekf_simulador_oclusao.png`
"""

import csv
import glob
import json
import math
import os
import sys

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
VOOS_DIR = os.path.join(os.path.dirname(__file__), 'resultados', 'voos_ekf_simulador')
RESULTADOS_DIR = os.path.join(os.path.dirname(__file__), 'resultados')
GRAFICOS_DIR = os.path.join(os.path.dirname(__file__), 'graficos')


def _wrap_deg(ang):
    """Normaliza diferença angular para [-180, 180) graus."""
    return (ang + 180.0) % 360.0 - 180.0


def _jitter_2d(segmentos):
    """
    Estima o jitter de alta frequência via segunda diferença (imune à tendência
    de velocidade constante durante a aproximação):
        sigma_jitter = sqrt(mean((d2x)^2 + (d2y)^2) / 6)
    e retorna também o RMS do salto quadro a quadro sqrt(mean(dx^2 + dy^2)).
    """
    d2_sq = []
    d1_sq = []
    for seg in segmentos:
        if len(seg) < 4:
            continue
        for k in range(1, len(seg)):
            dx1 = seg[k][0] - seg[k - 1][0]
            dy1 = seg[k][1] - seg[k - 1][1]
            d1_sq.append(dx1 * dx1 + dy1 * dy1)
        for k in range(1, len(seg) - 1):
            d2x = seg[k + 1][0] - 2.0 * seg[k][0] + seg[k - 1][0]
            d2y = seg[k + 1][1] - 2.0 * seg[k][1] + seg[k - 1][1]
            d2_sq.append(d2x * d2x + d2y * d2y)
    if not d2_sq:
        return None, None
    sigma_jitter = math.sqrt(sum(d2_sq) / (6.0 * len(d2_sq)))
    rms_passo = math.sqrt(sum(d1_sq) / len(d1_sq))
    return sigma_jitter, rms_passo


def _jitter_yaw_circular(segmentos_deg):
    """Calcula jitter circular em S^1 via segunda diferença de ângulos em graus."""
    d2_sq = []
    d1_sq = []
    for seg in segmentos_deg:
        if len(seg) < 4:
            continue
        diffs1 = [_wrap_deg(seg[k] - seg[k - 1]) for k in range(1, len(seg))]
        d1_sq.extend(d * d for d in diffs1)
        for k in range(1, len(diffs1)):
            d2 = _wrap_deg(diffs1[k] - diffs1[k - 1])
            d2_sq.append(d2 * d2)
    if not d2_sq:
        return None, None
    return math.sqrt(sum(d2_sq) / (6.0 * len(d2_sq))), math.sqrt(sum(d1_sq) / len(d1_sq))


def _extrair_segmentos_xy(linhas, filtro_fn):
    """Agrupa sequências contíguas válidas de pontos (bruto, ekf) para evitar cortes de fase."""
    segs_bruto = []
    segs_ekf = []
    cur_b = []
    cur_e = []
    ultima_etapa = None

    for r in linhas:
        ok = (
            filtro_fn(r)
            and r['Oclusao_Ativa'] == '0'
            and r['Visivel_YOLO'] == '1'
            and r['Erro_X_YOLO_Bruto_mm'] != ''
            and r['Erro_X_EKF_mm'] != ''
        )
        # Quebra segmento se trocar modelo (0.35m) para não contar reset como jitter
        etapa_base = r['Etapa'].split(':')[0].split('|')[0].strip()
        if ok and (ultima_etapa is None or etapa_base == ultima_etapa):
            cur_b.append((float(r['Erro_X_YOLO_Bruto_mm']), float(r['Erro_Y_YOLO_Bruto_mm'])))
            cur_e.append((float(r['Erro_X_EKF_mm']), float(r['Erro_Y_EKF_mm'])))
        else:
            if cur_b:
                segs_bruto.append(cur_b)
                segs_ekf.append(cur_e)
                cur_b, cur_e = [], []
            if ok:
                cur_b.append((float(r['Erro_X_YOLO_Bruto_mm']), float(r['Erro_Y_YOLO_Bruto_mm'])))
                cur_e.append((float(r['Erro_X_EKF_mm']), float(r['Erro_Y_EKF_mm'])))
        ultima_etapa = etapa_base if ok else None

    if cur_b:
        segs_bruto.append(cur_b)
        segs_ekf.append(cur_e)
    return segs_bruto, segs_ekf


def _extrair_segmentos_yaw(linhas):
    """Agrupa sequências contíguas válidas de Yaw bruto vs. EKF na Etapa 2."""
    segs_b, segs_e = [], []
    cur_b, cur_e = [], []
    for r in linhas:
        if r['Yaw_Bruto_deg'] != '' and r['Yaw_EKF_deg'] != '':
            cur_b.append(float(r['Yaw_Bruto_deg']))
            cur_e.append(float(r['Yaw_EKF_deg']))
        else:
            if cur_b:
                segs_b.append(cur_b)
                segs_e.append(cur_e)
                cur_b, cur_e = [], []
    if cur_b:
        segs_b.append(cur_b)
        segs_e.append(cur_e)
    return segs_b, segs_e


def _calc_reducao(bruto, filtrado):
    if bruto is None or filtrado is None or bruto <= 0.0:
        return None
    return (1.0 - (filtrado / bruto)) * 100.0


def analisar_voo_teste2_simulador(caminho_csv, caminho_marcos=None, gerar_grafico=True):
    """Processa um voo isolado do Teste 2 no Gazebo, exporta resumo CSV e gera gráfico."""
    os.makedirs(RESULTADOS_DIR, exist_ok=True)
    os.makedirs(GRAFICOS_DIR, exist_ok=True)

    with open(caminho_csv, 'r', encoding='utf-8') as f:
        linhas = list(csv.DictReader(f))

    marcos = {}
    if caminho_marcos and os.path.exists(caminho_marcos):
        with open(caminho_marcos, 'r', encoding='utf-8') as f:
            marcos = json.load(f)

    # 1. Atenuação de jitter XY em voo real (Fase 1, Fase 2 e combinada)
    f1_b, f1_e = _extrair_segmentos_xy(linhas, lambda r: 'FASE 1' in r['Fase'])
    f2_b, f2_e = _extrair_segmentos_xy(linhas, lambda r: 'FASE 2' in r['Fase'])

    jit_f1_b, passo_f1_b = _jitter_2d(f1_b)
    jit_f1_e, passo_f1_e = _jitter_2d(f1_e)
    jit_f2_b, passo_f2_b = _jitter_2d(f2_b)
    jit_f2_e, passo_f2_e = _jitter_2d(f2_e)
    jit_all_b, _ = _jitter_2d(f1_b + f2_b)
    jit_all_e, _ = _jitter_2d(f1_e + f2_e)

    # 2. Atenuação de jitter de Yaw (Etapa 2)
    yaw_segs_b, yaw_segs_e = _extrair_segmentos_yaw(linhas)
    jit_yaw_b, _ = _jitter_yaw_circular(yaw_segs_b)
    jit_yaw_e, _ = _jitter_yaw_circular(yaw_segs_e)

    # 3. Métricas da janela de oclusão total no simulador
    idx_occ = [i for i, r in enumerate(linhas) if r['Oclusao_Ativa'] == '1']
    ciclos_occ = len(idx_occ)
    ciclos_sustentados = sum(1 for i in idx_occ if linhas[i]['Erro_X_EKF_mm'] != '')
    taxa_sust_pct = (ciclos_sustentados / ciclos_occ * 100.0) if ciclos_occ else 0.0

    duracao_occ_s = 0.0
    alt_ini_occ = None
    alt_fim_occ = None
    ekf_fim_occ_mm = None
    reacq_bruto_mm = None
    reacq_ekf_1s_mm = None

    if idx_occ:
        i0, i1 = idx_occ[0], idx_occ[-1]
        t0 = float(linhas[i0]['Tempo_Relativo_s'])
        t1 = float(linhas[i1]['Tempo_Relativo_s'])
        duracao_occ_s = round(t1 - t0, 3)
        alt_ini_occ = float(linhas[i0]['Altitude_m'])
        alt_fim_occ = float(linhas[i1]['Altitude_m'])
        if linhas[i1]['Erro_XY_EKF_mm'] != '':
            ekf_fim_occ_mm = float(linhas[i1]['Erro_XY_EKF_mm'])

        # Primeiro frame visível após oclusão e estabilização +1.0 s depois
        for j in range(i1 + 1, len(linhas)):
            r = linhas[j]
            if reacq_bruto_mm is None and r['Visivel_YOLO'] == '1' and r['Erro_XY_YOLO_Bruto_mm'] != '':
                reacq_bruto_mm = float(r['Erro_XY_YOLO_Bruto_mm'])
            if float(r['Tempo_Relativo_s']) >= t1 + 1.0 and r['Erro_XY_EKF_mm'] != '':
                reacq_ekf_1s_mm = float(r['Erro_XY_EKF_mm'])
                break

    # 4. Marcos reais no mundo Gazebo (Ground Truth)
    eventos = marcos.get('eventos', {})
    ev_pre = eventos.get('pre_oclusao', {})
    ev_pos = eventos.get('pos_oclusao', {})
    ev_reacq = eventos.get('pos_reaquisicao', {})
    ev_pouso = eventos.get('pos_pouso', {})

    desvio_real_occ_mm = None
    if 'drone_x_m' in ev_pre and 'drone_x_m' in ev_pos:
        desvio_real_occ_mm = round(
            math.hypot(
                (ev_pos['drone_x_m'] - ev_pre['drone_x_m']) * 1000.0,
                (ev_pos['drone_y_m'] - ev_pre['drone_y_m']) * 1000.0,
            ),
            2,
        )

    resumo = {
        'Arquivo_Voo': os.path.basename(caminho_csv),
        'Timestamp': marcos.get('timestamp', ''),
        'Jitter_F1_Bruto_mm': round(jit_f1_b, 2) if jit_f1_b is not None else '',
        'Jitter_F1_EKF_mm': round(jit_f1_e, 2) if jit_f1_e is not None else '',
        'Reducao_Jitter_F1_pct': round(_calc_reducao(jit_f1_b, jit_f1_e), 1) if _calc_reducao(jit_f1_b, jit_f1_e) is not None else '',
        'Jitter_F2_Bruto_mm': round(jit_f2_b, 2) if jit_f2_b is not None else '',
        'Jitter_F2_EKF_mm': round(jit_f2_e, 2) if jit_f2_e is not None else '',
        'Reducao_Jitter_F2_pct': round(_calc_reducao(jit_f2_b, jit_f2_e), 1) if _calc_reducao(jit_f2_b, jit_f2_e) is not None else '',
        'Reducao_Jitter_Global_pct': round(_calc_reducao(jit_all_b, jit_all_e), 1) if _calc_reducao(jit_all_b, jit_all_e) is not None else '',
        'Jitter_Yaw_Bruto_deg': round(jit_yaw_b, 3) if jit_yaw_b is not None else '',
        'Jitter_Yaw_EKF_deg': round(jit_yaw_e, 3) if jit_yaw_e is not None else '',
        'Reducao_Jitter_Yaw_pct': round(_calc_reducao(jit_yaw_b, jit_yaw_e), 1) if _calc_reducao(jit_yaw_b, jit_yaw_e) is not None else '',
        'Ciclos_Oclusao': ciclos_occ,
        'Ciclos_EKF_Sustentados': ciclos_sustentados,
        'Sustentacao_Oclusao_pct': round(taxa_sust_pct, 1),
        'Duracao_Oclusao_s': duracao_occ_s,
        'Alt_Inicio_Oclusao_m': alt_ini_occ if alt_ini_occ is not None else '',
        'Alt_Fim_Oclusao_m': alt_fim_occ if alt_fim_occ is not None else '',
        'EKF_Erro_Fim_Oclusao_mm': round(ekf_fim_occ_mm, 2) if ekf_fim_occ_mm is not None else '',
        'Reaquisicao_YOLO_mm': round(reacq_bruto_mm, 2) if reacq_bruto_mm is not None else '',
        'Reaquisicao_EKF_1s_mm': round(reacq_ekf_1s_mm, 2) if reacq_ekf_1s_mm is not None else '',
        'Gazebo_Erro_Pre_Oclusao_mm': ev_pre.get('erro_radial_mundo_mm', ''),
        'Gazebo_Erro_Pos_Oclusao_mm': ev_pos.get('erro_radial_mundo_mm', ''),
        'Gazebo_Deslocamento_Oclusao_mm': desvio_real_occ_mm if desvio_real_occ_mm is not None else '',
        'Gazebo_Erro_Pos_Reaquisicao_mm': ev_reacq.get('erro_radial_mundo_mm', ''),
        'Gazebo_RMSE_Final_Pouso_mm': ev_pouso.get('erro_radial_mundo_mm', ''),
        'Passo_F1_Bruto_mm': round(passo_f1_b, 2) if passo_f1_b is not None else '',
        'Passo_F1_EKF_mm': round(passo_f1_e, 2) if passo_f1_e is not None else '',
        'Passo_F2_Bruto_mm': round(passo_f2_b, 2) if passo_f2_b is not None else '',
        'Passo_F2_EKF_mm': round(passo_f2_e, 2) if passo_f2_e is not None else '',
    }

    # Atualiza CSV resumo dedicado (sem duplicar o mesmo arquivo de voo)
    caminho_resumo = os.path.join(RESULTADOS_DIR, 'teste2_simulador_resumo.csv')
    campos = list(resumo.keys())
    existentes = []
    if os.path.exists(caminho_resumo) and os.path.getsize(caminho_resumo) > 0:
        with open(caminho_resumo, 'r', encoding='utf-8') as f:
            existentes = [r for r in csv.DictReader(f) if r.get('Arquivo_Voo') != resumo['Arquivo_Voo']]
    existentes.append(resumo)
    with open(caminho_resumo, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=campos)
        w.writeheader()
        w.writerows(existentes)

    caminho_png = None
    if gerar_grafico:
        caminho_png = _plotar_voo_oclusao(linhas, resumo, marcos)

    _imprimir_resumo(resumo, caminho_csv, caminho_resumo, caminho_png)
    return resumo, caminho_resumo, caminho_png


def _plotar_voo_oclusao(linhas, resumo, marcos):
    """Gera gráfico 3x1 mostrando atenuação de jitter no Gazebo, janela cega EKF e Yaw."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError as e:
        print(f"[!] Aviso: matplotlib/numpy indisponível para gerar gráfico ({e}). CSVs salvos normalmente.")
        return None

    t = np.array([float(r['Tempo_Relativo_s']) for r in linhas])
    alt = np.array([float(r['Altitude_m']) for r in linhas])
    occ = np.array([int(r['Oclusao_Ativa']) for r in linhas])
    exy_b = np.array([float(r['Erro_XY_YOLO_Bruto_mm']) if r['Erro_XY_YOLO_Bruto_mm'] != '' else np.nan for r in linhas])
    exy_e = np.array([float(r['Erro_XY_EKF_mm']) if r['Erro_XY_EKF_mm'] != '' else np.nan for r in linhas])
    vx = np.array([float(r['Vel_X_cmd']) for r in linhas])
    vy = np.array([float(r['Vel_Y_cmd']) for r in linhas])
    yaw_b = np.array([float(r['Yaw_Bruto_deg']) if r['Yaw_Bruto_deg'] != '' else np.nan for r in linhas])
    yaw_e = np.array([float(r['Yaw_EKF_deg']) if r['Yaw_EKF_deg'] != '' else np.nan for r in linhas])

    idx_occ = np.where(occ == 1)[0]
    t_occ_ini = float(t[idx_occ[0]]) if len(idx_occ) else None
    t_occ_fim = float(t[idx_occ[-1]]) if len(idx_occ) else None

    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle(
        'Validação do EKF no Simulador Gazebo: Atenuação de Jitter Real e Predição Cega em Oclusão',
        fontsize=12, fontweight='bold', y=0.98
    )

    # Painel 1: Foco na Fase 2 (perto da janela de oclusão, ou voo completo se não houver oclusão)
    ax1 = axes[0]
    mask_f2 = np.array(['FASE 2' in r['Fase'] for r in linhas])
    if t_occ_ini is not None:
        t_zoom_ini = max(float(t[mask_f2][0]) if np.any(mask_f2) else 0.0, t_occ_ini - 8.0)
        t_zoom_fim = min(float(t[-1]), t_occ_fim + 8.0)
        m = (t >= t_zoom_ini) & (t <= t_zoom_fim)
    else:
        m = mask_f2 if np.any(mask_f2) else np.ones_like(t, dtype=bool)

    ax1.plot(t[m], exy_b[m], 'o', color='#e74c3c', markersize=3.5, alpha=0.65, label='Visão Bruta YOLO + Pinhole (sem EKF)')
    ax1.plot(t[m], exy_e[m], '-', color='#1f77b4', linewidth=2.2, label='Saída EKF (filtragem + predição cega)')
    if t_occ_ini is not None:
        ax1.axvspan(t_occ_ini, t_occ_fim, color='#f39c12', alpha=0.25, label=f'Oclusão Simulada ({resumo["Duracao_Oclusao_s"]:.2f} s)')
    ax1.set_ylabel('Erro Radial XY (mm)', fontweight='bold')
    ax1.set_title(
        f'Fase 2 (Zoom na Oclusão) — Redução de Jitter F1: {resumo["Reducao_Jitter_F1_pct"]}% | F2: {resumo["Reducao_Jitter_F2_pct"]}%',
        fontsize=10
    )
    ax1.legend(loc='upper right', fontsize=8.5, frameon=True)
    ax1.grid(True, linestyle=':', alpha=0.7)

    # Painel 2: Altitude e Velocidades comandadas durante a Fase 2 (estabilidade da trajetória)
    ax2 = axes[1]
    ax2.plot(t[m], alt[m], color='#2ca02c', linewidth=2.2, label='Altitude (m)')
    if t_occ_ini is not None:
        ax2.axvspan(t_occ_ini, t_occ_fim, color='#f39c12', alpha=0.25)
    ax2.set_ylabel('Altitude (m)', color='#2ca02c', fontweight='bold')
    ax2.tick_params(axis='y', labelcolor='#2ca02c')
    ax2_v = ax2.twinx()
    ax2_v.plot(t[m], vx[m], '--', color='#34495e', linewidth=1.3, alpha=0.85, label='Vel X cmd (m/s)')
    ax2_v.plot(t[m], vy[m], ':', color='#8e44ad', linewidth=1.5, alpha=0.85, label='Vel Y cmd (m/s)')
    ax2_v.set_ylabel('Comandos PD (m/s)', color='#34495e', fontweight='bold')
    ax2.set_xlabel('Tempo de Missão (s)', fontweight='bold')
    ax2.set_title('Estabilidade da Trajetória Vertical e dos Comandos de Velocidade Durante Oclusão', fontsize=10)
    h1, l1 = ax2.get_legend_handles_labels()
    h2, l2 = ax2_v.get_legend_handles_labels()
    ax2.legend(h1 + h2, l1 + l2, loc='upper right', fontsize=8.5, frameon=True)
    ax2.grid(True, linestyle=':', alpha=0.7)

    # Painel 3: Yaw Bruto vs EKF na Etapa 2
    ax3 = axes[2]
    m_yaw = ~np.isnan(yaw_b) | ~np.isnan(yaw_e)
    if np.any(m_yaw):
        ax3.plot(t[m_yaw], yaw_b[m_yaw], 'o', color='#e67e22', markersize=3.5, alpha=0.6, label='Yaw Bruto (3 alças YOLO)')
        ax3.plot(t[m_yaw], yaw_e[m_yaw], '-', color='#6c3483', linewidth=2.0, label='Yaw Filtrado (FiltroAnguloEKF)')
        ax3.axhline(1.0, color='gray', linestyle=':', alpha=0.7)
        ax3.axhline(-1.0, color='gray', linestyle=':', alpha=0.7, label='Faixa Meta ±1,0°')
        ax3.set_title(
            f'Etapa 2 (Alinhamento de Yaw @ 1,50 m) — Atenuação de Ruído Angular: {resumo["Reducao_Jitter_Yaw_pct"]}%',
            fontsize=10
        )
        ax3.legend(loc='upper right', fontsize=8.5, frameon=True)
    else:
        ax3.text(0.5, 0.5, 'Sem amostras de Yaw registradas', ha='center', va='center', transform=ax3.transAxes)
    ax3.set_ylabel('Erro de Yaw (graus)', fontweight='bold')
    ax3.set_xlabel('Tempo de Missão (s)', fontweight='bold')
    ax3.grid(True, linestyle=':', alpha=0.7)

    plt.tight_layout(rect=[0, 0.02, 1, 0.95])
    caminho_png = os.path.join(GRAFICOS_DIR, 'teste2_ekf_simulador_oclusao.png')
    plt.savefig(caminho_png, dpi=300)
    plt.close()
    return caminho_png


def _imprimir_resumo(resumo, caminho_csv, caminho_resumo, caminho_png):
    print("\n" + "=" * 74)
    print("🧪 RESULTADO ISOLADO — TESTE 2 NO SIMULADOR GAZEBO (EKF + OCLUSÃO)")
    print("=" * 74)
    print(f" Arquivo do voo             : {resumo['Arquivo_Voo']}")
    print(f" Atenuação Jitter XY (F1)   : {resumo['Jitter_F1_Bruto_mm']} mm -> {resumo['Jitter_F1_EKF_mm']} mm (-{resumo['Reducao_Jitter_F1_pct']}%)")
    print(f" Atenuação Jitter XY (F2)   : {resumo['Jitter_F2_Bruto_mm']} mm -> {resumo['Jitter_F2_EKF_mm']} mm (-{resumo['Reducao_Jitter_F2_pct']}%)")
    print(f" Atenuação Jitter Yaw (S1)  : {resumo['Jitter_Yaw_Bruto_deg']}° -> {resumo['Jitter_Yaw_EKF_deg']}° (-{resumo['Reducao_Jitter_Yaw_pct']}%)")
    print("-" * 74)
    print(f" Janela de Oclusão Simulada : {resumo['Duracao_Oclusao_s']} s ({resumo['Ciclos_EKF_Sustentados']}/{resumo['Ciclos_Oclusao']} ciclos sustentados = {resumo['Sustentacao_Oclusao_pct']}%)")
    print(f" Altitude (Início -> Fim)   : {resumo['Alt_Inicio_Oclusao_m']} m -> {resumo['Alt_Fim_Oclusao_m']} m")
    print(f" Erro Gazebo Pré-Oclusão    : {resumo['Gazebo_Erro_Pre_Oclusao_mm']} mm")
    print(f" Erro Gazebo Pós-Oclusão    : {resumo['Gazebo_Erro_Pos_Oclusao_mm']} mm (Deslocamento real na oclusão: {resumo['Gazebo_Deslocamento_Oclusao_mm']} mm)")
    print(f" Reaquisição Visão / +1.0s  : YOLO={resumo['Reaquisicao_YOLO_mm']} mm | EKF+1s={resumo['Reaquisicao_EKF_1s_mm']} mm")
    print(f" Pouso Final Pós-LAND (Gz)  : {resumo['Gazebo_RMSE_Final_Pouso_mm']} mm")
    print("-" * 74)
    print(f" [+] Série temporal salva   : {caminho_csv}")
    print(f" [+] Resumo CSV Teste 2     : {caminho_resumo}")
    if caminho_png:
        print(f" [+] Gráfico gerado         : {caminho_png}")
    print(" [i] Isolamento garantido   : relatorio_testes_monte_carlo.csv e Code/missao/telemetria/ intocados.")
    print("=" * 74 + "\n")


def main():
    if len(sys.argv) > 1:
        caminho_csv = sys.argv[1]
    else:
        arquivos = sorted(glob.glob(os.path.join(VOOS_DIR, 'teste2_voo_*.csv')))
        if not arquivos:
            print(f"[!] Nenhum voo do Teste 2 encontrado em: {VOOS_DIR}")
            print("    Rode primeiro no simulador: python3 Code/missao/main.py --teste-oclusao")
            sys.exit(1)
        caminho_csv = max(arquivos, key=os.path.getmtime)

    caminho_marcos = caminho_csv.replace('.csv', '_marcos.json')
    analisar_voo_teste2_simulador(caminho_csv, caminho_marcos)


if __name__ == '__main__':
    main()
