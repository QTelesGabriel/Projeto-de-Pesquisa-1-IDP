#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gera o gráfico de dispersão espacial dos pousos nos ensaios de Monte Carlo (Figura 4.3).
Lê os registros reais em Code/missao/relatorio_testes_monte_carlo.csv.
"""

import os
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CSV_PATH = os.path.join(BASE_DIR, 'Code', 'missao', 'relatorio_testes_monte_carlo.csv')
OUTPUT_DIR = os.path.join(BASE_DIR, 'Autonomous_Object_Capture_System_By_UAVs_Using_Visual_Servoing', 'figuras')
OUTPUT_FIG = os.path.join(OUTPUT_DIR, 'dispersao_pousos_monte_carlo.png')

def gerar_grafico():
    xs, ys, rmses, sucessos = [], [], [], []
    xs_pre, ys_pre = [], []

    if not os.path.exists(CSV_PATH):
        print(f"[!] Erro: Arquivo {CSV_PATH} não encontrado.")
        return

    with open(CSV_PATH, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            ex = float(row['T5_Erro_X_mm'])
            ey = float(row['T5_Erro_Y_mm'])
            rmse = float(row['T5_RMSE_Final_mm'])
            suc = row['T5_Acoplamento_Confirmado'].strip().upper() == 'SIM'
            xs.append(ex)
            ys.append(ey)
            rmses.append(rmse)
            sucessos.append(suc)
            if row.get('T5_Erro_X_PreBlindDrop_mm'):
                xs_pre.append(float(row['T5_Erro_X_PreBlindDrop_mm']))
                ys_pre.append(float(row['T5_Erro_Y_PreBlindDrop_mm']))

    xs = np.array(xs)
    ys = np.array(ys)
    sucessos = np.array(sucessos)

    fig, ax = plt.subplots(figsize=(8.5, 7.5), dpi=300)

    # Círculos concêntricos de referência
    theta = np.linspace(0, 2*np.pi, 200)
    
    # Tolerância de projeto (8 mm)
    ax.plot(8.0 * np.cos(theta), 8.0 * np.sin(theta), color='#27ae60', linestyle='--', linewidth=2.0,
            label='Tolerância Estrita de Projeto ($r = 8,0$ mm)', zorder=2)
    ax.fill(8.0 * np.cos(theta), 8.0 * np.sin(theta), color='#2ecc71', alpha=0.08, zorder=1)

    # RMSE médio terminal (5.25 mm)
    ax.plot(5.25 * np.cos(theta), 5.25 * np.sin(theta), color='#2980b9', linestyle=':', linewidth=1.8,
            label='RMSE Médio Terminal Pós-Pouso ($5,25$ mm)', zorder=2)

    # RMSE pré-blind drop (3.13 mm)
    ax.plot(3.13 * np.cos(theta), 3.13 * np.sin(theta), color='#8e44ad', linestyle='-.', linewidth=1.5,
            label='RMSE Médio Pré-Blind Drop ($3,13$ mm)', zorder=2)

    # Pontos de pouso
    ax.scatter(xs[sucessos], ys[sucessos], color='#27ae60', edgecolors='#1e8449', s=55, alpha=0.85,
               marker='o', label=f'Sucesso Estrito ($N = {np.sum(sucessos)}$, 76,7%)', zorder=4)
    ax.scatter(xs[~sucessos], ys[~sucessos], color='#e74c3c', edgecolors='#c0392b', s=65, alpha=0.9,
               marker='^', label=f'Desvio por Efeito Solo ($N = {np.sum(~sucessos)}$, 23,3%)', zorder=4)

    # Centro do alvo
    ax.plot(0, 0, marker='+', color='black', markersize=12, markeredgewidth=2, label='Centro da Armadilha ($0, 0$)', zorder=5)

    ax.set_aspect('equal', 'box')
    ax.set_xlim(-13, 13)
    ax.set_ylim(-13, 13)
    ax.set_xlabel('Desvio no Eixo X Terminal ($e_x$, mm)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Desvio no Eixo Y Terminal ($e_y$, mm)', fontsize=11, fontweight='bold')
    ax.set_title('Diagrama de Dispersão Métrico dos Pousos Terminais no Solo\n(Bateria de Monte Carlo, $N = 60$ Missões Autônomas)',
                 fontsize=12, fontweight='bold', pad=15)

    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='lower left', fontsize=8.5, framealpha=0.95)

    plt.tight_layout()
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plt.savefig(OUTPUT_FIG, dpi=300)
    plt.close()
    print(f"[+] Gráfico de dispersão salvo com sucesso em: {OUTPUT_FIG}")

if __name__ == '__main__':
    gerar_grafico()
