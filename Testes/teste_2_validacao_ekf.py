#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Teste 2: Validação Estocástica Monte Carlo do Filtro de Kalman Estendido (EKF)
Atende ao Objetivo Específico 2 do TCC:
  1. Atenuação quantitativa de ruído (jitter e outliers) na posição métrica XY e no ângulo Yaw.
  2. Validação da continuidade angular sobre a variedade circular S^1 cruzando ±180° (±pi).
  3. Sustentação preditiva contínua por exatamente 4,0 segundos (80 ciclos a 20 Hz) de oclusão.
  4. Comportamento determinístico de corte (reset após > 4,0 s) e reinicialização limpa pós-timeout.

Metodologia científica:
  - Executa N = 50 ensaios estocásticos independentes (Monte Carlo) sobre perfis físicos
    compatíveis com o modelo cinemático e a aproximação real do VANT (deriva translacional
    residual + micro-perturbação aerodinâmica + rotação cruzando a fronteira ±180°).
  - Calcula Média, Desvio Padrão, Mínimo e Máximo das métricas, aplicando critérios objetivos
    programáticos de aprovação (sem resultados fixos).
"""

import os
import sys
import math
import csv
import numpy as np
import matplotlib.pyplot as plt

# Adiciona o caminho do código da missão para importar as classes reais do EKF
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(ROOT_DIR, 'Code', 'missao'))

from visao.filtro import FiltroAlvoEKF, FiltroAnguloEKF

RESULTADOS_DIR = os.path.join(os.path.dirname(__file__), 'resultados')
GRAFICOS_DIR = os.path.join(os.path.dirname(__file__), 'graficos')
os.makedirs(RESULTADOS_DIR, exist_ok=True)
os.makedirs(GRAFICOS_DIR, exist_ok=True)

# Parâmetros do protocolo experimental (20 Hz)
DT = 0.05
TEMPO_TOTAL = 17.5
NUM_PASSOS = int(round(TEMPO_TOTAL / DT))  # 350 passos exatos
NUM_ENSAIOS_MC = 50

# Janelas temporais do protocolo (índices exatos)
IDX_OCLUSAO1_INI = int(round(5.0 / DT))   # 100 (t = 5.00 s)
IDX_OCLUSAO1_FIM = int(round(9.0 / DT))   # 180 (t = 9.00 s -> 80 passos = 4.00 s)
IDX_OCLUSAO2_INI = int(round(11.5 / DT))  # 230 (t = 11.50 s)
IDX_TIMEOUT_EXATO = IDX_OCLUSAO2_INI + 80 # 310 (t = 15.50 s: último passo <= 4.00 s; 311 = 4.05 s -> reset)
IDX_OCLUSAO2_FIM = int(round(16.0 / DT))  # 320 (t = 16.00 s -> retorno visual pós-timeout)


def executar_ensaio_unico(seed):
    """
    Executa um ensaio estocástico completo (17,5 s a 20 Hz) utilizando diretamente
    FiltroAlvoEKF e FiltroAnguloEKF.
    """
    rng = np.random.default_rng(seed)
    t = np.arange(NUM_PASSOS) * DT

    # 1. Trajetória real fisicamente coerente (deriva residual + micro-perturbação suave)
    vx_real = rng.uniform(-0.06, 0.06)
    vy_real = rng.uniform(-0.06, 0.06)
    x0 = rng.uniform(-0.20, 0.20)
    y0 = rng.uniform(-0.20, 0.20)

    x_real = x0 + vx_real * t + 0.012 * np.sin(0.35 * t)
    y_real = y0 + vy_real * t + 0.012 * np.cos(0.30 * t)

    # Orientação Yaw cruzando intencionalmente a fronteira ±180° (±pi) em S^1
    yaw0_rad = math.radians(rng.uniform(155.0, 170.0))
    omega_real = math.radians(rng.uniform(3.0, 5.0))
    yaw_real_cont = yaw0_rad + omega_real * t
    yaw_real = np.array([FiltroAnguloEKF.normalizar_angulo(a) for a in yaw_real_cont])

    # 2. Injeção de ruído de visão (jitter gaussiano + outliers ocasionais)
    ruido_x = rng.normal(0.0, 0.035, size=NUM_PASSOS)
    ruido_y = rng.normal(0.0, 0.035, size=NUM_PASSOS)
    ruido_yaw = rng.normal(0.0, math.radians(3.5), size=NUM_PASSOS)

    mascara_visivel = np.ones(NUM_PASSOS, dtype=bool)
    mascara_visivel[IDX_OCLUSAO1_INI:IDX_OCLUSAO1_FIM] = False
    mascara_visivel[IDX_OCLUSAO2_INI:IDX_OCLUSAO2_FIM] = False

    for i in range(5, NUM_PASSOS, 25):
        if mascara_visivel[i]:
            ruido_x[i] += rng.choice([-0.12, 0.12])
            ruido_y[i] += rng.choice([-0.10, 0.10])
            ruido_yaw[i] += rng.choice([-math.radians(10.0), math.radians(10.0)])

    x_medido = x_real + ruido_x
    y_medido = y_real + ruido_y
    yaw_medido = np.array([
        FiltroAnguloEKF.normalizar_angulo(a) for a in (yaw_real + ruido_yaw)
    ])

    # 3. Execução das instâncias reais do EKF
    filtro_alvo = FiltroAlvoEKF(dt_inicial=DT)
    filtro_alvo.timeout_limite = 4.0

    filtro_yaw = FiltroAnguloEKF(dt_inicial=DT)
    filtro_yaw.timeout_limite = 4.0

    x_ekf = np.full(NUM_PASSOS, np.nan)
    y_ekf = np.full(NUM_PASSOS, np.nan)
    yaw_ekf = np.full(NUM_PASSOS, np.nan)
    erro_xy_ekf_mm = np.full(NUM_PASSOS, np.nan)
    erro_xy_bruto_mm = np.full(NUM_PASSOS, np.nan)
    erro_yaw_ekf_deg = np.full(NUM_PASSOS, np.nan)
    erro_yaw_bruto_deg = np.full(NUM_PASSOS, np.nan)

    sustentou_4s = True
    timeout_exato_ok = True
    reset_limpo_ok = True

    for i in range(NUM_PASSOS):
        if mascara_visivel[i]:
            zx, zy, zyaw = float(x_medido[i]), float(y_medido[i]), float(yaw_medido[i])
            erro_xy_bruto_mm[i] = math.hypot(zx - x_real[i], zy - y_real[i]) * 1000.0
            erro_yaw_bruto_deg[i] = abs(math.degrees(
                FiltroAnguloEKF.normalizar_angulo(zyaw - yaw_real[i])
            ))
        else:
            zx, zy, zyaw = None, None, None

        x_est, y_est = filtro_alvo.atualizar(zx, zy, DT)
        yaw_est = filtro_yaw.atualizar(zyaw, DT)

        if x_est is not None and y_est is not None:
            x_ekf[i] = x_est
            y_ekf[i] = y_est
            erro_xy_ekf_mm[i] = math.hypot(x_est - x_real[i], y_est - y_real[i]) * 1000.0

        if yaw_est is not None:
            yaw_ekf[i] = yaw_est
            erro_yaw_ekf_deg[i] = abs(math.degrees(
                FiltroAnguloEKF.normalizar_angulo(yaw_est - yaw_real[i])
            ))

        # Checagem rigorosa da janela de sustentação de 4,0 s (80 ciclos: indices 100..179)
        if IDX_OCLUSAO1_INI <= i < IDX_OCLUSAO1_FIM:
            if x_est is None or yaw_est is None:
                sustentou_4s = False

        # Checagem determinística da janela de timeout (> 4,0 s)
        # De IDX_OCLUSAO2_INI (230) até IDX_TIMEOUT_EXATO - 1 (309): 80 passos <= 4,00 s -> deve sustentar
        if IDX_OCLUSAO2_INI <= i < IDX_TIMEOUT_EXATO:
            if x_est is None or yaw_est is None:
                timeout_exato_ok = False
        # A partir do 81º passo sem visão (i = 310, 4,05 s sem visão) até 319: deve resetar (retornar None)
        elif IDX_TIMEOUT_EXATO <= i < IDX_OCLUSAO2_FIM:
            if x_est is not None or yaw_est is not None:
                timeout_exato_ok = False
            if filtro_alvo.iniciado or filtro_yaw.iniciado:
                reset_limpo_ok = False

    # 4. Métricas por ensaio
    idx_ativo = np.arange(0, IDX_OCLUSAO1_INI)
    rmse_xy_bruto = float(np.sqrt(np.mean(erro_xy_bruto_mm[idx_ativo] ** 2)))
    rmse_xy_ekf = float(np.sqrt(np.mean(erro_xy_ekf_mm[idx_ativo] ** 2)))
    reducao_xy_pct = (1.0 - (rmse_xy_ekf / rmse_xy_bruto)) * 100.0

    rmse_yaw_bruto = float(np.sqrt(np.mean(erro_yaw_bruto_deg[idx_ativo] ** 2)))
    rmse_yaw_ekf = float(np.sqrt(np.mean(erro_yaw_ekf_deg[idx_ativo] ** 2)))
    reducao_yaw_pct = (1.0 - (rmse_yaw_ekf / rmse_yaw_bruto)) * 100.0

    # Deriva ao final exato dos 4,0 s de oclusão (último passo cego: IDX_OCLUSAO1_FIM - 1)
    idx_fim_occ1 = IDX_OCLUSAO1_FIM - 1
    deriva_xy_4s_mm = float(erro_xy_ekf_mm[idx_fim_occ1])
    deriva_yaw_4s_deg = float(erro_yaw_ekf_deg[idx_fim_occ1])

    # Erro 1,0 s após reaquisição da câmera (t = 10,0 s)
    idx_reacq_1s = IDX_OCLUSAO1_FIM + int(round(1.0 / DT))
    erro_reacq_1s_mm = float(np.sqrt(np.mean(
        erro_xy_ekf_mm[idx_reacq_1s:IDX_OCLUSAO2_INI] ** 2
    )))

    # Convergência pós-timeout (últimos 0,5 s após retorno visual em t = 16,0 s)
    idx_pos_conv = IDX_OCLUSAO2_FIM + int(round(1.0 / DT))
    erro_pos_timeout_mm = float(np.sqrt(np.mean(
        erro_xy_ekf_mm[idx_pos_conv:] ** 2
    )))
    reaquisicao_pos_timeout_ok = (
        not np.isnan(erro_pos_timeout_mm) and erro_pos_timeout_mm <= 80.0
    )

    return {
        't': t,
        'x_real': x_real,
        'y_real': y_real,
        'yaw_real_cont': yaw_real_cont,
        'yaw_real': yaw_real,
        'x_medido': x_medido,
        'y_medido': y_medido,
        'yaw_medido': yaw_medido,
        'mascara_visivel': mascara_visivel,
        'x_ekf': x_ekf,
        'y_ekf': y_ekf,
        'yaw_ekf': yaw_ekf,
        'erro_xy_bruto_mm': erro_xy_bruto_mm,
        'erro_xy_ekf_mm': erro_xy_ekf_mm,
        'rmse_xy_bruto_mm': rmse_xy_bruto,
        'rmse_xy_ekf_mm': rmse_xy_ekf,
        'reducao_xy_pct': reducao_xy_pct,
        'rmse_yaw_bruto_deg': rmse_yaw_bruto,
        'rmse_yaw_ekf_deg': rmse_yaw_ekf,
        'reducao_yaw_pct': reducao_yaw_pct,
        'deriva_xy_4s_mm': deriva_xy_4s_mm,
        'deriva_yaw_4s_deg': deriva_yaw_4s_deg,
        'erro_reacq_1s_mm': erro_reacq_1s_mm,
        'erro_pos_timeout_mm': erro_pos_timeout_mm,
        'sustentou_4s': sustentou_4s,
        'timeout_exato_ok': (timeout_exato_ok and reset_limpo_ok),
        'reaquisicao_pos_timeout_ok': reaquisicao_pos_timeout_ok,
    }


def _stats(valores):
    arr = np.asarray(valores, dtype=float)
    return {
        'media': float(np.mean(arr)),
        'std': float(np.std(arr)),
        'min': float(np.min(arr)),
        'max': float(np.max(arr)),
    }


def executar_monte_carlo(n_ensaios=NUM_ENSAIOS_MC):
    ensaios = [executar_ensaio_unico(seed=100 + i) for i in range(n_ensaios)]

    resumo = {
        'n_ensaios': n_ensaios,
        'rmse_xy_bruto_mm': _stats([e['rmse_xy_bruto_mm'] for e in ensaios]),
        'rmse_xy_ekf_mm': _stats([e['rmse_xy_ekf_mm'] for e in ensaios]),
        'reducao_xy_pct': _stats([e['reducao_xy_pct'] for e in ensaios]),
        'rmse_yaw_bruto_deg': _stats([e['rmse_yaw_bruto_deg'] for e in ensaios]),
        'rmse_yaw_ekf_deg': _stats([e['rmse_yaw_ekf_deg'] for e in ensaios]),
        'reducao_yaw_pct': _stats([e['reducao_yaw_pct'] for e in ensaios]),
        'deriva_xy_4s_mm': _stats([e['deriva_xy_4s_mm'] for e in ensaios]),
        'deriva_yaw_4s_deg': _stats([e['deriva_yaw_4s_deg'] for e in ensaios]),
        'erro_reacq_1s_mm': _stats([e['erro_reacq_1s_mm'] for e in ensaios]),
        'erro_pos_timeout_mm': _stats([e['erro_pos_timeout_mm'] for e in ensaios]),
        'taxa_reducao_xy_positiva_pct': 100.0 * sum(1 for e in ensaios if e['reducao_xy_pct'] > 0.0) / n_ensaios,
        'taxa_sustentacao_4s_pct': 100.0 * sum(1 for e in ensaios if e['sustentou_4s']) / n_ensaios,
        'taxa_timeout_exato_pct': 100.0 * sum(1 for e in ensaios if e['timeout_exato_ok']) / n_ensaios,
        'taxa_reaquisicao_pos_timeout_pct': 100.0 * sum(1 for e in ensaios if e['reaquisicao_pos_timeout_ok']) / n_ensaios,
    }

    # Critérios objetivos programáticos (sem resultados hardcoded)
    criterios = {
        'reducao_xy_ok': (
            resumo['reducao_xy_pct']['media'] >= 40.0
            and resumo['taxa_reducao_xy_positiva_pct'] == 100.0
        ),
        'reducao_yaw_ok': (resumo['reducao_yaw_pct']['media'] >= 40.0),
        'sustentacao_4s_ok': (resumo['taxa_sustentacao_4s_pct'] == 100.0),
        'deriva_4s_ok': (resumo['deriva_xy_4s_mm']['media'] <= 250.0),
        'reacq_ok': (resumo['erro_reacq_1s_mm']['media'] <= 60.0),
        'timeout_ok': (resumo['taxa_timeout_exato_pct'] == 100.0),
        'pos_timeout_ok': (resumo['taxa_reaquisicao_pos_timeout_pct'] == 100.0),
    }

    return ensaios, resumo, criterios


def _alinhar_em_ramo_continuo(angulos_rad, ref_cont_rad):
    """Alinha ângulos em [-pi, pi] ao ramo contínuo da referência, preservando NaNs."""
    out = np.full_like(angulos_rad, np.nan, dtype=float)
    for i in range(len(angulos_rad)):
        if np.isfinite(angulos_rad[i]):
            diff = FiltroAnguloEKF.normalizar_angulo(angulos_rad[i] - ref_cont_rad[i])
            out[i] = math.degrees(ref_cont_rad[i] + diff)
    return out


def _adicionar_faixas_janelas(ax, mostrar_legenda=False):
    lbl1 = 'Oclusão Transitória (4,0 s exatos)' if mostrar_legenda else None
    lbl2 = 'Timeout > 4,0 s + Retorno em 16,0 s' if mostrar_legenda else None
    ax.axvspan(5.0, 9.0, color='orange', alpha=0.18, label=lbl1)
    ax.axvspan(11.5, 16.0, color='red', alpha=0.12, label=lbl2)
    ax.axvline(15.50, color='crimson', linestyle=':', linewidth=1.3,
               label=('Reset Determinístico (t = 15,50 s)' if mostrar_legenda else None))


def plotar_resultados(ensaios, resumo):
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, axes = plt.subplots(2, 2, figsize=(14, 9.5), sharex=True)
    ax_x, ax_y = axes[0, 0], axes[0, 1]
    ax_yaw, ax_mc = axes[1, 0], axes[1, 1]

    fig.suptitle(
        f'Validação Estocástica Monte Carlo do Filtro de Kalman Estendido (EKF — N = {resumo["n_ensaios"]} ensaios)\n'
        'Atenuação de Jitter, Continuidade em S¹, Predição Cega por 4,0 s e Reset Determinístico (Objetivo 2)',
        fontsize=12.5, fontweight='bold', y=0.98
    )

    rep = ensaios[0]
    t = rep['t']
    vis = rep['mascara_visivel']

    # (a) Posição X
    ax_x.plot(t, rep['x_real'], 'k--', linewidth=1.7, label='Trajetória Real (Ground Truth)')
    ax_x.plot(t[vis], rep['x_medido'][vis], 'r.', markersize=3.5, alpha=0.45, label='Medição Visual YOLO (Jitter + Outliers)')
    ax_x.plot(t, rep['x_ekf'], 'b-', linewidth=2.0, label='Estimativa EKF')
    _adicionar_faixas_janelas(ax_x, mostrar_legenda=True)
    ax_x.set_ylabel('Posição X (m)', fontsize=10, fontweight='bold')
    ax_x.set_title(
        f'(a) Eixo X (Ensaio Rep.) — Redução XY: {resumo["reducao_xy_pct"]["media"]:.1f}% ± {resumo["reducao_xy_pct"]["std"]:.1f}%',
        fontsize=10
    )
    ax_x.legend(loc='best', frameon=True, fontsize=7.5)
    ax_x.grid(True, linestyle=':', alpha=0.7)

    # (b) Posição Y
    ax_y.plot(t, rep['y_real'], 'k--', linewidth=1.7, label='Trajetória Real')
    ax_y.plot(t[vis], rep['y_medido'][vis], 'r.', markersize=3.5, alpha=0.45, label='Medição Visual')
    ax_y.plot(t, rep['y_ekf'], 'b-', linewidth=2.0, label='Estimativa EKF')
    _adicionar_faixas_janelas(ax_y)
    ax_y.set_ylabel('Posição Y (m)', fontsize=10, fontweight='bold')
    ax_y.set_title(
        f'(b) Eixo Y (Ensaio Rep.) — Deriva após 4,0 s: {resumo["deriva_xy_4s_mm"]["media"]:.1f} ± {resumo["deriva_xy_4s_mm"]["std"]:.1f} mm',
        fontsize=10
    )
    ax_y.grid(True, linestyle=':', alpha=0.7)

    # (c) Orientação Yaw cruzando fronteira ±180° em S^1
    yaw_real_deg = np.degrees(rep['yaw_real_cont'])
    yaw_med_deg = _alinhar_em_ramo_continuo(rep['yaw_medido'], rep['yaw_real_cont'])
    yaw_ekf_deg = _alinhar_em_ramo_continuo(rep['yaw_ekf'], rep['yaw_real_cont'])

    ax_yaw.plot(t, yaw_real_deg, 'k--', linewidth=1.7, label='Orientação Real')
    ax_yaw.plot(t[vis], yaw_med_deg[vis], 'g.', markersize=3.5, alpha=0.45, label='Leitura Angular Bruta')
    ax_yaw.plot(t, yaw_ekf_deg, 'purple', linewidth=2.0, label='Yaw Filtrado EKF (S¹)')
    ax_yaw.axhline(180.0, color='magenta', linestyle='-.', linewidth=1.2, alpha=0.85, label='Fronteira ±180° (±π rad)')
    _adicionar_faixas_janelas(ax_yaw)
    ax_yaw.set_ylabel('Ângulo Yaw contínuo (graus)', fontsize=10, fontweight='bold')
    ax_yaw.set_xlabel('Tempo Decorrido (segundos)', fontsize=10, fontweight='bold')
    ax_yaw.set_title(
        f'(c) Cruzamento ±180° em S¹ — RMSE Yaw: {resumo["rmse_yaw_bruto_deg"]["media"]:.2f}° → {resumo["rmse_yaw_ekf_deg"]["media"]:.2f}° ({resumo["reducao_yaw_pct"]["media"]:.1f}%)',
        fontsize=10
    )
    ax_yaw.legend(loc='best', frameon=True, fontsize=7.5)
    ax_yaw.grid(True, linestyle=':', alpha=0.7)

    # (d) Erro Radial Monte Carlo (Média ± 1 Desvio Padrão sobre N=50 ensaios)
    mat_raw = np.array([e['erro_xy_bruto_mm'] for e in ensaios])
    mat_ekf = np.array([e['erro_xy_ekf_mm'] for e in ensaios])

    raw_media = np.full(NUM_PASSOS, np.nan)
    raw_std = np.full(NUM_PASSOS, np.nan)
    ekf_media = np.full(NUM_PASSOS, np.nan)
    ekf_std = np.full(NUM_PASSOS, np.nan)

    for k in range(NUM_PASSOS):
        v_raw = mat_raw[:, k]
        v_raw = v_raw[np.isfinite(v_raw)]
        if len(v_raw) > 0:
            raw_media[k] = np.mean(v_raw)
            raw_std[k] = np.std(v_raw)

        v_ekf = mat_ekf[:, k]
        v_ekf = v_ekf[np.isfinite(v_ekf)]
        if len(v_ekf) > 0:
            ekf_media[k] = np.mean(v_ekf)
            ekf_std[k] = np.std(v_ekf)

    ax_mc.plot(t, raw_media, color='tab:red', linewidth=1.4, alpha=0.75, label='Erro Visão Bruta (Média)')
    ax_mc.fill_between(t, np.maximum(0.0, raw_media - raw_std), raw_media + raw_std, color='tab:red', alpha=0.12)
    ax_mc.plot(t, ekf_media, color='tab:blue', linewidth=2.2, label='Erro EKF (Média ± 1σ)')
    ax_mc.fill_between(t, np.maximum(0.0, ekf_media - ekf_std), ekf_media + ekf_std, color='tab:blue', alpha=0.22)
    _adicionar_faixas_janelas(ax_mc)
    ax_mc.set_ylabel('Erro Radial XY (mm)', fontsize=10, fontweight='bold')
    ax_mc.set_xlabel('Tempo Decorrido (segundos)', fontsize=10, fontweight='bold')
    ax_mc.set_title(
        f'(d) Curva Estatística Monte Carlo (N = {resumo["n_ensaios"]}) — Erro Médio & Reaquisição',
        fontsize=10
    )
    ax_mc.legend(loc='upper left', frameon=True, fontsize=7.5)
    ax_mc.grid(True, linestyle=':', alpha=0.7)

    plt.tight_layout(rect=[0, 0.02, 1, 0.94])
    caminho_fig = os.path.join(GRAFICOS_DIR, 'validacao_ekf_4segundos.png')
    plt.savefig(caminho_fig, dpi=300)
    plt.close()
    print(f"[+] Gráfico científico salvo em: {caminho_fig}")


def exportar_csv(resumo, criterios):
    caminho_csv = os.path.join(RESULTADOS_DIR, 'validacao_ekf_metricas.csv')
    linhas = [
        ('RMSE_Posicao_Bruta', resumo['rmse_xy_bruto_mm'], 'mm', 'N/A (Baseline)', 'N/A'),
        ('RMSE_Posicao_EKF', resumo['rmse_xy_ekf_mm'], 'mm', '< Baseline',
         'SIM' if resumo['rmse_xy_ekf_mm']['media'] < resumo['rmse_xy_bruto_mm']['media'] else 'NAO'),
        ('Reducao_Jitter_Posicao', resumo['reducao_xy_pct'], '%', '>= 40.0% em 100% ensaios',
         'SIM' if criterios['reducao_xy_ok'] else 'NAO'),
        ('RMSE_Yaw_Bruto', resumo['rmse_yaw_bruto_deg'], 'graus', 'N/A (Baseline)', 'N/A'),
        ('RMSE_Yaw_EKF', resumo['rmse_yaw_ekf_deg'], 'graus', '< Baseline',
         'SIM' if resumo['rmse_yaw_ekf_deg']['media'] < resumo['rmse_yaw_bruto_deg']['media'] else 'NAO'),
        ('Reducao_Ruido_Yaw_S1', resumo['reducao_yaw_pct'], '%', '>= 40.0%',
         'SIM' if criterios['reducao_yaw_ok'] else 'NAO'),
        ('Deriva_Posicao_4s_Predicao', resumo['deriva_xy_4s_mm'], 'mm', '<= 250.0 mm',
         'SIM' if criterios['deriva_4s_ok'] else 'NAO'),
        ('Deriva_Yaw_4s_Predicao', resumo['deriva_yaw_4s_deg'], 'graus', 'Informativo', 'N/A'),
        ('Erro_Reaquisicao_Pos_Oclusao', resumo['erro_reacq_1s_mm'], 'mm', '<= 60.0 mm',
         'SIM' if criterios['reacq_ok'] else 'NAO'),
        ('Erro_Pos_Reset_Timeout', resumo['erro_pos_timeout_mm'], 'mm', '<= 80.0 mm em 100% ensaios',
         'SIM' if criterios['pos_timeout_ok'] else 'NAO'),
    ]

    with open(caminho_csv, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Metrica', 'Media', 'Desvio_Padrao', 'Minimo', 'Maximo',
            'Unidade', 'Criterio_Objetivo', 'Meta_TCC_Atendida'
        ])
        for nome, st, unid, crit, ok in linhas:
            writer.writerow([
                nome,
                f"{st['media']:.2f}", f"{st['std']:.2f}",
                f"{st['min']:.2f}", f"{st['max']:.2f}",
                unid, crit, ok
            ])
        writer.writerow([
            'Sustentacao_4s_Sem_Divergencia',
            f"{resumo['taxa_sustentacao_4s_pct']:.1f}", '0.00',
            f"{resumo['taxa_sustentacao_4s_pct']:.1f}", f"{resumo['taxa_sustentacao_4s_pct']:.1f}",
            '%', '100.0% (80/80 ciclos)', 'SIM' if criterios['sustentacao_4s_ok'] else 'NAO'
        ])
        writer.writerow([
            'Timeout_Deterministico_Pos_4s',
            f"{resumo['taxa_timeout_exato_pct']:.1f}", '0.00',
            f"{resumo['taxa_timeout_exato_pct']:.1f}", f"{resumo['taxa_timeout_exato_pct']:.1f}",
            '%', '100.0% reset exato > 4.0s', 'SIM' if criterios['timeout_ok'] else 'NAO'
        ])
    print(f"[+] Métricas do EKF exportadas para: {caminho_csv}")


def main():
    print("\n" + "=" * 82)
    print("🎯 TESTE 2: VALIDAÇÃO MONTE CARLO DO FILTRO DE KALMAN ESTENDIDO (EKF)")
    print(f"   Protocolo: N = {NUM_ENSAIOS_MC} ensaios estocásticos a 20 Hz (com cruzamento ±180° em S¹)")
    print("=" * 82)

    ensaios, resumo, criterios = executar_monte_carlo(NUM_ENSAIOS_MC)

    print("\n[+] 1. Atenuação de Jitter com Visão Ativa (0,0 s a 5,0 s):")
    print(f"    * RMSE XY (Visão Bruta)       : {resumo['rmse_xy_bruto_mm']['media']:.2f} ± {resumo['rmse_xy_bruto_mm']['std']:.2f} mm")
    print(f"    * RMSE XY (EKF Filtrado)      : {resumo['rmse_xy_ekf_mm']['media']:.2f} ± {resumo['rmse_xy_ekf_mm']['std']:.2f} mm")
    print(f"    -> REDUÇÃO DE JITTER XY       : {resumo['reducao_xy_pct']['media']:.1f} ± {resumo['reducao_xy_pct']['std']:.1f}% [{'ATENDIDO' if criterios['reducao_xy_ok'] else 'NAO ATENDIDO'}]")
    print("-" * 70)
    print(f"    * RMSE Yaw S¹ (Bruto)         : {resumo['rmse_yaw_bruto_deg']['media']:.2f} ± {resumo['rmse_yaw_bruto_deg']['std']:.2f}°")
    print(f"    * RMSE Yaw S¹ (EKF Filtrado)  : {resumo['rmse_yaw_ekf_deg']['media']:.2f} ± {resumo['rmse_yaw_ekf_deg']['std']:.2f}°")
    print(f"    -> REDUÇÃO DE RUÍDO YAW (S¹)  : {resumo['reducao_yaw_pct']['media']:.1f} ± {resumo['reducao_yaw_pct']['std']:.1f}% [{'ATENDIDO' if criterios['reducao_yaw_ok'] else 'NAO ATENDIDO'}]")
    print("-" * 70)
    print("[+] 2. Oclusão Transitória de Exatos 4,00 s (80 ciclos sem detecção):")
    print(f"    * Sustentação Contínua (80/80): {resumo['taxa_sustentacao_4s_pct']:.1f}% dos ensaios [{'ATENDIDO' if criterios['sustentacao_4s_ok'] else 'NAO ATENDIDO'}]")
    print(f"    * Deriva XY ao Final dos 4,0s : {resumo['deriva_xy_4s_mm']['media']:.1f} ± {resumo['deriva_xy_4s_mm']['std']:.1f} mm (máx: {resumo['deriva_xy_4s_mm']['max']:.1f} mm) [{'ATENDIDO' if criterios['deriva_4s_ok'] else 'NAO ATENDIDO'}]")
    print(f"    * Deriva Yaw ao Final dos 4,0s: {resumo['deriva_yaw_4s_deg']['media']:.2f} ± {resumo['deriva_yaw_4s_deg']['std']:.2f}°")
    print(f"    * Reaquisição Pós-Oclusão     : {resumo['erro_reacq_1s_mm']['media']:.1f} ± {resumo['erro_reacq_1s_mm']['std']:.1f} mm [{'ATENDIDO' if criterios['reacq_ok'] else 'NAO ATENDIDO'}]")
    print("-" * 70)
    print("[+] 3. Timeout de Segurança (> 4,0 s) e Reinicialização Limpa:")
    print(f"    * Corte Exato após 4,00 s     : {resumo['taxa_timeout_exato_pct']:.1f}% dos ensaios [{'ATENDIDO' if criterios['timeout_ok'] else 'NAO ATENDIDO'}]")
    print(f"    * Convergência Pós-Reset      : {resumo['erro_pos_timeout_mm']['media']:.1f} ± {resumo['erro_pos_timeout_mm']['std']:.1f} mm ({resumo['taxa_reaquisicao_pos_timeout_pct']:.1f}% ok) [{'ATENDIDO' if criterios['pos_timeout_ok'] else 'NAO ATENDIDO'}]")
    print("=" * 82)

    exportar_csv(resumo, criterios)
    plotar_resultados(ensaios, resumo)


if __name__ == "__main__":
    main()
