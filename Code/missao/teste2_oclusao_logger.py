# -*- coding: utf-8 -*-
"""
Logger dedicado e isolado para validação do EKF (Teste 2) em voo real no Gazebo.

Grava estritamente em `Testes/resultados/voos_ekf_simulador/`, sem criar arquivos
em `Code/missao/telemetria/` nem alterar `relatorio_testes_monte_carlo.csv`.
"""

import csv
import json
import math
import os
import threading
import time
from datetime import datetime


ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
DEFAULT_OUT_DIR = os.path.join(ROOT_DIR, 'Testes', 'resultados', 'voos_ekf_simulador')


class Teste2OclusaoLogger:
    """Registra série temporal bruto-vs-EKF, oclusão simulada e marcos no Gazebo."""

    COLUNAS = [
        'Timestamp', 'Tempo_Relativo_s', 'Fase', 'Etapa', 'Altitude_m',
        'Oclusao_Ativa', 'Visivel_YOLO', 'EKF_Predizendo',
        'Erro_X_YOLO_Bruto_mm', 'Erro_Y_YOLO_Bruto_mm', 'Erro_XY_YOLO_Bruto_mm',
        'Erro_X_EKF_mm', 'Erro_Y_EKF_mm', 'Erro_XY_EKF_mm',
        'Yaw_Bruto_deg', 'Yaw_EKF_deg',
        'Vel_X_cmd', 'Vel_Y_cmd', 'Vel_Z_cmd', 'Yaw_Rate_cmd',
        'Drone_Local_N_m', 'Drone_Local_E_m', 'Drone_Local_D_m',
    ]

    def __init__(self, diretorio_saida=None, condicoes_iniciais=None):
        self.diretorio_saida = diretorio_saida or DEFAULT_OUT_DIR
        os.makedirs(self.diretorio_saida, exist_ok=True)

        self.timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.nome_csv = f"teste2_voo_{self.timestamp_str}.csv"
        self.nome_marcos = f"teste2_voo_{self.timestamp_str}_marcos.json"
        self.caminho_csv = os.path.join(self.diretorio_saida, self.nome_csv)
        self.caminho_marcos = os.path.join(self.diretorio_saida, self.nome_marcos)

        self._lock = threading.Lock()
        self.tempo_inicio = time.time()
        self._contador = 0
        self.marcos = {
            'timestamp': self.timestamp_str,
            'condicoes_iniciais': condicoes_iniciais or {},
            'eventos': {},
        }

        self.arquivo = open(self.caminho_csv, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.arquivo)
        self.writer.writerow(self.COLUNAS)
        self.arquivo.flush()

    @staticmethod
    def extrair_local_ned(vehicle):
        """Extrai coordenadas locais (N, E, D) do veículo de forma não-bloqueante."""
        loc = getattr(getattr(vehicle, 'location', None), 'local_frame', None)
        if loc is None or loc.north is None or loc.east is None or loc.down is None:
            return None, None, None
        return float(loc.north), float(loc.east), float(loc.down)

    @staticmethod
    def _fmt_mm(val_m):
        return f"{val_m * 1000.0:.2f}" if val_m is not None else ""

    @staticmethod
    def _fmt_radial_mm(x_m, y_m):
        if x_m is None or y_m is None:
            return ""
        return f"{math.hypot(x_m, y_m) * 1000.0:.2f}"

    def registrar(
        self,
        fase,
        etapa,
        altitude_m,
        vehicle=None,
        oclusao_ativa=False,
        visivel_yolo=False,
        ekf_predizendo=False,
        erro_x_bruto_m=None,
        erro_y_bruto_m=None,
        erro_x_ekf_m=None,
        erro_y_ekf_m=None,
        yaw_bruto_deg=None,
        yaw_ekf_deg=None,
        vel_x=0.0,
        vel_y=0.0,
        vel_z=0.0,
        yaw_rate=0.0,
    ):
        """Grava uma amostra síncrona (20 Hz) da comparação Bruto vs. EKF."""
        loc_n, loc_e, loc_d = self.extrair_local_ned(vehicle) if vehicle is not None else (None, None, None)
        linha = [
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            f"{time.time() - self.tempo_inicio:.3f}",
            fase,
            etapa,
            f"{max(0.0, float(altitude_m)):.3f}",
            1 if oclusao_ativa else 0,
            1 if visivel_yolo else 0,
            1 if ekf_predizendo else 0,
            self._fmt_mm(erro_x_bruto_m),
            self._fmt_mm(erro_y_bruto_m),
            self._fmt_radial_mm(erro_x_bruto_m, erro_y_bruto_m),
            self._fmt_mm(erro_x_ekf_m),
            self._fmt_mm(erro_y_ekf_m),
            self._fmt_radial_mm(erro_x_ekf_m, erro_y_ekf_m),
            f"{yaw_bruto_deg:.2f}" if yaw_bruto_deg is not None else "",
            f"{yaw_ekf_deg:.2f}" if yaw_ekf_deg is not None else "",
            f"{float(vel_x):.3f}",
            f"{float(vel_y):.3f}",
            f"{float(vel_z):.3f}",
            f"{float(yaw_rate):.3f}",
            f"{loc_n:.4f}" if loc_n is not None else "",
            f"{loc_e:.4f}" if loc_e is not None else "",
            f"{loc_d:.4f}" if loc_d is not None else "",
        ]
        with self._lock:
            if not self.arquivo or self.arquivo.closed:
                return
            self.writer.writerow(linha)
            self._contador += 1
            if self._contador % 10 == 0:
                self.arquivo.flush()

    def registrar_marco(self, nome_marco, poses_mundo=None, extra=None):
        """Salva um marco de Ground Truth (ex.: pré-oclusão, pós-oclusão, pós-pouso)."""
        payload = {
            'tempo_relativo_s': round(time.time() - self.tempo_inicio, 3),
        }
        if poses_mundo is not None:
            pos_d = poses_mundo['drone_coaxial']['position']
            pos_a = poses_mundo['armadilha_alvo']['position']
            dx_mm = (pos_d['x'] - pos_a['x']) * 1000.0
            dy_mm = (pos_d['y'] - pos_a['y']) * 1000.0
            payload.update({
                'drone_x_m': round(pos_d['x'], 5),
                'drone_y_m': round(pos_d['y'], 5),
                'drone_z_m': round(pos_d['z'], 5),
                'armadilha_x_m': round(pos_a['x'], 5),
                'armadilha_y_m': round(pos_a['y'], 5),
                'dx_mundo_mm': round(dx_mm, 2),
                'dy_mundo_mm': round(dy_mm, 2),
                'erro_radial_mundo_mm': round(math.hypot(dx_mm, dy_mm), 2),
            })
        if extra:
            payload.update(extra)
        with self._lock:
            self.marcos['eventos'][nome_marco] = payload
            with open(self.caminho_marcos, 'w', encoding='utf-8') as f:
                json.dump(self.marcos, f, indent=2, ensure_ascii=False)

    def fechar(self):
        """Descarrega CSV e JSON de marcos no disco."""
        with self._lock:
            with open(self.caminho_marcos, 'w', encoding='utf-8') as f:
                json.dump(self.marcos, f, indent=2, ensure_ascii=False)
            if self.arquivo and not self.arquivo.closed:
                self.arquivo.flush()
                self.arquivo.close()
