# -*- coding: utf-8 -*-
"""
Módulo de Telemetria Científica Contínua.
Grava séries temporais completas de cada voo de simulação para
permitir a geração automática de gráficos e tabelas para a monografia.
"""

import os
import csv
import time
from datetime import datetime


class TelemetriaLogger:
    def __init__(self, diretorio_base=None):
        if diretorio_base is None:
            diretorio_base = os.path.join(os.path.dirname(__file__), 'telemetria')
        os.makedirs(diretorio_base, exist_ok=True)
        
        self.timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.nome_arquivo = f"telemetria_voo_{self.timestamp_str}.csv"
        self.caminho_completo = os.path.join(diretorio_base, self.nome_arquivo)
        
        self.tempo_inicio = time.time()
        self.arquivo = open(self.caminho_completo, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.arquivo)
        
        # Cabeçalho completo para análise científica
        self.writer.writerow([
            'Timestamp', 'Tempo_Relativo_s', 'Fase', 'Etapa',
            'Altitude_m', 'Erro_X_m', 'Erro_Y_m', 'Erro_XY_mm', 'Erro_Yaw_deg',
            'Vel_X_cmd', 'Vel_Y_cmd', 'Vel_Z_cmd', 'Yaw_Rate_cmd',
            'Afastamento_FOV_pct', 'Descendo_Flag', 'Bloqueios_Histerese', 'EKF_Ativo'
        ])
        self.arquivo.flush()
        self._contador_grava = 0

    def registrar(self, fase, etapa, altitude, erro_x=None, erro_y=None, 
                  erro_yaw_deg=None, vel_x=0.0, vel_y=0.0, vel_z=0.0, yaw_rate=0.0,
                  afastamento_fov_pct=0.0, descendo=False, bloqueios=0, ekf_ativo=True):
        """Registra uma linha de telemetria instantânea."""
        tempo_rel = round(time.time() - self.tempo_inicio, 3)
        
        err_x_str = f"{erro_x:.4f}" if erro_x is not None else ""
        err_y_str = f"{erro_y:.4f}" if erro_y is not None else ""
        if erro_x is not None and erro_y is not None:
            err_xy_mm = f"{( (erro_x**2 + erro_y**2)**0.5 ) * 1000.0:.2f}"
        else:
            err_xy_mm = ""
            
        err_yaw_str = f"{erro_yaw_deg:.2f}" if erro_yaw_deg is not None else ""
        
        self.writer.writerow([
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            tempo_rel,
            fase,
            etapa,
            round(altitude, 3),
            err_x_str,
            err_y_str,
            err_xy_mm,
            err_yaw_str,
            round(vel_x, 3),
            round(vel_y, 3),
            round(vel_z, 3),
            round(yaw_rate, 3),
            round(afastamento_fov_pct, 1),
            1 if descendo else 0,
            bloqueios,
            1 if ekf_ativo else 0
        ])
        
        self._contador_grava += 1
        if self._contador_grava % 10 == 0:
            self.arquivo.flush()

    def fechar(self):
        """Finaliza e descarrega o arquivo no disco."""
        if self.arquivo and not self.arquivo.closed:
            self.arquivo.flush()
            self.arquivo.close()
