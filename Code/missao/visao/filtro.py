# visao/filtro.py
# -*- coding: utf-8 -*-
"""
Módulo de Filtragem Estocástica de Estados - Filtro de Kalman Estendido (EKF)
Implementação recursiva não linear em conformidade com as Equações (2.1) a (2.8)
da Fundamentação Teórica da pesquisa de VANTs com Servovisão.

Contempla:
  1. FiltroAlvoEKF: Rastreamento métrico de posição e velocidade (dim_x=4, dim_z=2)
     com modelo cinemático, cálculo analítico da matriz Jacobiana de transição e
     observação direta.
  2. FiltroAnguloEKF: Rastreamento da orientação de guinada (Yaw) e velocidade angular
     (dim_x=2, dim_z=1) sobre a variedade circular S^1, com inovação angular não linear.
"""

import numpy as np
import math


class ExtendedKalmanFilterCore:
    """
    Núcleo matemático do Filtro de Kalman Estendido (EKF).
    Executa o ciclo recursivo em duas etapas principais:
      - Predição (a priori):
          x_pred = M(x)
          P_pred = A * P * A^T + Q
      - Atualização / Correção (a posteriori):
          y = z - h(x_pred)  (Inovação)
          S = H * P_pred * H^T + R  (Covariância residual)
          K = P_pred * H^T * S^(-1)  (Ganho de Kalman)
          x = x_pred + K * y
          P = (I - K*H) * P_pred * (I - K*H)^T + K * R * K^T  (Forma estabilizada de Joseph)
    """

    def __init__(self, dim_x, dim_z):
        self.dim_x = dim_x
        self.dim_z = dim_z

        # Vetor de estados (dim_x x 1)
        self.x = np.zeros((dim_x, 1), dtype=float)

        # Matriz de covariância do erro de estado (P)
        self.P = np.eye(dim_x, dtype=float)

        # Matriz de covariância do ruído de processo (Q)
        self.Q = np.eye(dim_x, dtype=float)

        # Matriz de covariância do ruído de medição (R)
        self.R = np.eye(dim_z, dtype=float)

        # Matriz Jacobiana de transição de estados (A / F)
        self.F = np.eye(dim_x, dtype=float)

        # Matriz Jacobiana do modelo de observação (H)
        self.H = np.zeros((dim_z, dim_x), dtype=float)

        # Vetor de inovação (y) e Ganho de Kalman (K)
        self.y = np.zeros((dim_z, 1), dtype=float)
        self.K = np.zeros((dim_x, dim_z), dtype=float)
        self.S = np.zeros((dim_z, dim_z), dtype=float)

        # Matriz identidade auxiliar para a forma de Joseph
        self._I = np.eye(dim_x, dtype=float)

    def predict(self, f_func=None, F_jacobian=None):
        """
        Etapa de Predição a priori do EKF.
        Aplica a função não linear de transição M(x) e projeta a covariância via Jacobiana A.
        """
        if f_func is not None:
            self.x = f_func(self.x)
        else:
            self.x = np.dot(self.F, self.x)

        if F_jacobian is not None:
            self.F = F_jacobian(self.x)

        # P = A * P * A^T + Q (Equação 2.4)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z, h_func=None, H_jacobian=None, residual_func=None):
        """
        Etapa de Correção a posteriori do EKF.
        Calcula a inovação, o ganho de Kalman e atualiza estado e covariância.
        """
        if h_func is not None:
            z_pred = h_func(self.x)
        else:
            z_pred = np.dot(self.H, self.x)

        if H_jacobian is not None:
            self.H = H_jacobian(self.x)

        # Cálculo da inovação (resíduo de medição)
        if residual_func is not None:
            self.y = residual_func(z, z_pred)
        else:
            self.y = z - z_pred

        # Covariância residual S = H * P * H^T + R
        self.S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R

        # Inversão numérica estável (Equação 2.5)
        try:
            inv_S = np.linalg.inv(self.S)
        except np.linalg.LinAlgError:
            inv_S = np.linalg.pinv(self.S)

        # Ganho de Kalman K = P * H^T * S^(-1)
        self.K = np.dot(np.dot(self.P, self.H.T), inv_S)

        # Correção a posteriori do estado x = x + K * y (Equação 2.6)
        self.x = self.x + np.dot(self.K, self.y)

        # Atualização da covariância usando a Forma Estabilizada de Joseph (Equação 2.7)
        # Garante simetria e positividade semidefinida contínua da matriz P
        I_KH = self._I - np.dot(self.K, self.H)
        self.P = np.dot(np.dot(I_KH, self.P), I_KH.T) + np.dot(np.dot(self.K, self.R), self.K.T)


class FiltroAlvoEKF:
    """
    Filtro de Kalman Estendido para rastreamento espacial do alvo em 2D métrico.
    Estado X: [x, y, vx, vy]^T (Posição métrica em metros e velocidades em m/s)
    Medição Z: [x, y]^T (Posição métrica fornecida pela visão / Pinhole)
    """

    def __init__(self, dt_inicial=0.1):
        self.kf = ExtendedKalmanFilterCore(dim_x=4, dim_z=2)

        # Estado Inicial [x, y, vx, vy]
        self.kf.x = np.array([[0.0], [0.0], [0.0], [0.0]])

        # Covariância inicial P (alta incerteza inicial antes da 1ª leitura)
        self.kf.P = np.eye(4) * 1000.0

        # Matriz de Ruído de Medição (R) calibrada para atenuar o jitter da detecção YOLO
        ruido_yolo = 0.5
        self.kf.R = np.array([
            [ruido_yolo, 0.0],
            [0.0, ruido_yolo]
        ])

        # Matriz de Ruído do Processo (Q)
        # Posição muda pouco (0.01) pela inércia física; velocidade admite variações por manobras (0.1)
        self.kf.Q = np.array([
            [0.01, 0.0,  0.0,  0.0],
            [0.0,  0.01, 0.0,  0.0],
            [0.0,  0.0,  0.1,  0.0],
            [0.0,  0.0,  0.0,  0.1]
        ])

        # Matriz Jacobiana de Observação (H_k = d(h)/dx)
        self.kf.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])

        self.iniciado = False
        self.dt_anterior = dt_inicial

        # Timeout de segurança para evitar predições cegas prolongadas
        self.tempo_sem_medicao = 0.0
        self.timeout_limite = 4.0  # Segundos sustentados de predição cega tolerada

    def reset(self):
        """
        Reseta o estado interno do EKF quando o alvo é perdido.
        Garante que, ao recuperar a visão da armadilha, o filtro reinicie suavemente sem arrasto residual.
        """
        self.iniciado = False
        self.tempo_sem_medicao = 0.0
        self.kf.x = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.kf.P = np.eye(4) * 1000.0

    def atualizar(self, z_x, z_y, dt):
        """
        Recebe a nova leitura métrica (z_x, z_y) e o intervalo de amostragem dt.
        Retorna a posição filtrada (x_limpo, y_limpo).
        """
        if dt <= 0:
            dt = self.dt_anterior

        # Inicialização no primeiro frame válido
        if not self.iniciado:
            if z_x is not None and z_y is not None:
                self.kf.x = np.array([[z_x], [z_y], [0.0], [0.0]])
                self.iniciado = True
                self.tempo_sem_medicao = 0.0
                return z_x, z_y
            else:
                return None, None

        # --- ETAPA 1: PREDIÇÃO DO EKF (Equações 2.3 e 2.4) ---
        # Matriz Jacobiana de transição de estado A_{k-1} avaliada no instante dt
        self.kf.F = np.array([
            [1.0, 0.0,  dt, 0.0],
            [0.0, 1.0, 0.0,  dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # Modelo cinemático de transição M(x)
        def modelo_transicao(x):
            return np.array([
                [x[0, 0] + x[2, 0] * dt],
                [x[1, 0] + x[3, 0] * dt],
                [x[2, 0]],
                [x[3, 0]]
            ])

        self.kf.predict(f_func=modelo_transicao, F_jacobian=lambda _: self.kf.F)

        # --- ETAPA 2: CORREÇÃO / ATUALIZAÇÃO DO EKF (Equações 2.5, 2.6 e 2.7) ---
        if z_x is not None and z_y is not None:
            z = np.array([[z_x], [z_y]])

            # Função de observação h(x) mapeia o vetor de estado para medição de posição
            def modelo_observacao(x):
                return np.array([
                    [x[0, 0]],
                    [x[1, 0]]
                ])

            self.kf.update(z, h_func=modelo_observacao, H_jacobian=lambda _: self.kf.H)
            self.tempo_sem_medicao = 0.0
        else:
            self.tempo_sem_medicao += dt

        self.dt_anterior = dt

        # Verificação do limite de segurança sem visão
        if self.tempo_sem_medicao > self.timeout_limite:
            self.iniciado = False
            return None, None

        x_limpo = float(self.kf.x[0, 0])
        y_limpo = float(self.kf.x[1, 0])
        return x_limpo, y_limpo


class FiltroAnguloEKF:
    """
    Filtro de Kalman Estendido para orientação angular de guinada (Yaw).
    Estado X: [theta, omega]^T (Ângulo em radianos e velocidade angular em rad/s)
    Medição Z: [theta]^T (Ângulo medido da armadilha em radianos)
    
    Trata formalmente a não linearidade da variedade circular S^1 (wrap-around de [-pi, pi]).
    """

    def __init__(self, dt_inicial=0.1):
        self.kf = ExtendedKalmanFilterCore(dim_x=2, dim_z=1)

        self.kf.x = np.array([[0.0], [0.0]])
        self.kf.P = np.eye(2) * 1000.0

        ruido_yolo_pose = 0.5
        self.kf.R = np.array([[ruido_yolo_pose]])

        self.kf.Q = np.array([
            [0.01, 0.0],
            [0.0,  0.1]
        ])

        # Jacobiana de Observação H_k = d(h)/dx
        self.kf.H = np.array([[1.0, 0.0]])

        self.iniciado = False
        self.dt_anterior = dt_inicial

        self.tempo_sem_medicao = 0.0
        self.timeout_limite = 4.0

    @staticmethod
    def normalizar_angulo(ang):
        """Normaliza qualquer ângulo contínuo para o intervalo [-pi, pi]."""
        return (ang + np.pi) % (2.0 * np.pi) - np.pi

    def reset(self):
        """Reseta o estado do filtro angular."""
        self.iniciado = False
        self.tempo_sem_medicao = 0.0
        self.kf.x = np.array([[0.0], [0.0]])
        self.kf.P = np.eye(2) * 1000.0

    def atualizar(self, z_theta, dt):
        """
        Recebe o ângulo medido z_theta em radianos e dt.
        Retorna o ângulo estimado theta_filtrado normalizado em [-pi, pi].
        """
        if dt <= 0:
            dt = self.dt_anterior

        if not self.iniciado:
            if z_theta is not None:
                self.kf.x = np.array([[self.normalizar_angulo(z_theta)], [0.0]])
                self.iniciado = True
                self.tempo_sem_medicao = 0.0
                return float(self.kf.x[0, 0])
            return None

        # --- ETAPA 1: PREDIÇÃO DO EKF ---
        # Matriz Jacobiana de transição A_{k-1}
        self.kf.F = np.array([
            [1.0, dt],
            [0.0, 1.0]
        ])

        # Transição cinemática não linear com normalização no círculo S^1
        def modelo_transicao_ang(x):
            theta_pred = self.normalizar_angulo(x[0, 0] + x[1, 0] * dt)
            omega_pred = x[1, 0]
            return np.array([[theta_pred], [omega_pred]])

        self.kf.predict(f_func=modelo_transicao_ang, F_jacobian=lambda _: self.kf.F)

        # --- ETAPA 2: CORREÇÃO / ATUALIZAÇÃO DO EKF ---
        if z_theta is not None:
            z = np.array([[self.normalizar_angulo(z_theta)]])

            # Função de observação h(x)
            def modelo_observacao_ang(x):
                return np.array([[self.normalizar_angulo(x[0, 0])]])

            # Resíduo de inovação circular em [-pi, pi]
            def residuo_circular(z_med, z_pred):
                diff = z_med[0, 0] - z_pred[0, 0]
                diff_norm = self.normalizar_angulo(diff)
                return np.array([[diff_norm]])

            self.kf.update(
                z,
                h_func=modelo_observacao_ang,
                H_jacobian=lambda _: self.kf.H,
                residual_func=residuo_circular
            )

            # Garante que o estado a posteriori permaneça contínuo em [-pi, pi]
            self.kf.x[0, 0] = self.normalizar_angulo(self.kf.x[0, 0])
            self.tempo_sem_medicao = 0.0
        else:
            self.tempo_sem_medicao += dt
            if self.tempo_sem_medicao > self.timeout_limite:
                self.iniciado = False
                return None

        self.dt_anterior = dt
        theta_filtrado = float(self.kf.x[0, 0])
        return theta_filtrado
