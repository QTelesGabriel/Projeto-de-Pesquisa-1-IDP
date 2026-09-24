import numpy as np
from filterpy.kalman import KalmanFilter

class FiltroAlvoEKF:
    def __init__(self, dt_inicial=0.1):
        """
        Inicializa o Filtro de Kalman.
        Estado X: [x, y, vx, vy]
        Medição Z: [x, y]
        """
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        
        # Estado Inicial (formato coluna 4x1)
        self.kf.x = np.array([[0.0], [0.0], [0.0], [0.0]])
        
        # Matriz de Covariância de Estado (P) - Quão incertos estamos do estado inicial
        self.kf.P *= 1000.0
        
        # Matriz de Ruído de Medição (R) - Quão barulhenta é a YOLO
        # Aumentado para 0.5 para segurar o tremor (jitter) dos Keypoints do modelo Pose Nano
        ruido_yolo = 0.5 
        self.kf.R = np.array([[ruido_yolo, 0.0],
                              [0.0, ruido_yolo]])
                               
        # Matriz de Ruído do Processo (Q)
        # Posição muda pouco (0.01) porque obedece à física. Velocidade muda mais (0.1) devido a aceleração.
        # Isso impede o "teletransporte" do alvo e cria um rastreio muito mais suave.
        self.kf.Q = np.array([
            [0.01, 0.0, 0.0, 0.0],
            [0.0, 0.01, 0.0, 0.0],
            [0.0, 0.0, 0.1, 0.0],
            [0.0, 0.0, 0.0, 0.1]
        ]) 
        
        # Matriz de Observação (H) - Como extraímos a medição Z a partir do estado X
        self.kf.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])
        
        self.iniciado = False
        self.dt_anterior = dt_inicial
        
        # Timeout para evitar predições fantasmas longas (Latência aceitável)
        self.tempo_sem_medicao = 0.0
        self.timeout_limite = 4.0  # Segundos (Aumentado para segurar predição por mais tempo sem YOLO)

    def atualizar(self, z_x, z_y, dt):
        """
        Recebe a nova leitura da YOLO e o tempo desde a última leitura (dt).
        Retorna a coordenada X e Y filtrada.
        """
        if dt <= 0:
            dt = self.dt_anterior

        # Se for a primeira leitura da YOLO, apenas iniciamos o estado na posição lida
        if not self.iniciado:
            if z_x is not None and z_y is not None:
                self.kf.x = np.array([[z_x], [z_y], [0.0], [0.0]])
                self.iniciado = True
                self.tempo_sem_medicao = 0.0
                return z_x, z_y
            else:
                return None, None # Ainda não temos nada para iniciar
            
        # --- PASSO 1: PREDIÇÃO ---
        # Matriz de Transição de Estado (F)
        self.kf.F = np.array([
            [1.0, 0.0,  dt, 0.0],
            [0.0, 1.0, 0.0,  dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        
        self.kf.predict()
        
        # --- PASSO 2: ATUALIZAÇÃO (com a medição da YOLO) ---
        if z_x is not None and z_y is not None:
            z = np.array([[z_x], [z_y]])
            self.kf.update(z)
            self.tempo_sem_medicao = 0.0 # Zera o cronômetro pois vimos o alvo
        else:
            self.tempo_sem_medicao += dt # Soma o tempo que ficamos "cegos"
            
        self.dt_anterior = dt
        
        # Lógica de Timeout (Ponto 1)
        if self.tempo_sem_medicao > self.timeout_limite:
            self.iniciado = False # Reseta o filtro para evitar voar em predição cega longa
            return None, None
        
        # Retorna apenas as coordenadas x e y limpas
        x_limpo = float(self.kf.x[0, 0])
        y_limpo = float(self.kf.x[1, 0])
        return x_limpo, y_limpo

import math

class FiltroAnguloEKF:
    def __init__(self, dt_inicial=0.1):
        """
        Inicializa o Filtro de Kalman para 1 Dimensão (Ângulo).
        Estado X: [theta, omega] (ângulo, velocidade angular)
        Medição Z: [theta]
        """
        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        
        self.kf.x = np.array([[0.0], [0.0]])
        self.kf.P *= 1000.0
        
        ruido_yolo_pose = 0.5 
        self.kf.R = np.array([[ruido_yolo_pose]])
                               
        self.kf.Q = np.array([
            [0.01, 0.0],
            [0.0, 0.1]
        ]) 
        
        self.kf.H = np.array([[1.0, 0.0]])
        
        self.iniciado = False
        self.dt_anterior = dt_inicial
        
        self.tempo_sem_medicao = 0.0
        self.timeout_limite = 4.0

    def atualizar(self, z_theta, dt):
        if dt <= 0:
            dt = self.dt_anterior

        if not self.iniciado:
            if z_theta is not None:
                self.kf.x = np.array([[z_theta], [0.0]])
                self.iniciado = True
            return z_theta

        self.kf.F = np.array([
            [1.0, dt],
            [0.0, 1.0]
        ])

        if z_theta is not None:
            # Precisamos tratar o wrap-around (salto brusco) do ângulo!
            # Se o ângulo pulou de -3.14 para +3.14, o erro cruza a fronteira.
            # Então, vamos corrigir a medição Z para ficar perto do estado atual.
            estado_theta_atual = self.kf.x[0, 0]
            diferenca = z_theta - estado_theta_atual
            # Normaliza a diferença para o intervalo [-PI, PI]
            diferenca = (diferenca + np.pi) % (2 * np.pi) - np.pi
            
            # Z corrigido é o estado atual + a menor diferença física
            z_corrigido = estado_theta_atual + diferenca

            self.tempo_sem_medicao = 0.0
            self.kf.predict()
            self.kf.update(np.array([[z_corrigido]]))
        else:
            self.tempo_sem_medicao += dt
            if self.tempo_sem_medicao > self.timeout_limite:
                self.iniciado = False
                return None
            self.kf.predict()

        self.dt_anterior = dt
        x_pred = self.kf.x
        
        # Normaliza o ângulo de saída filtrado para [-PI, PI]
        theta_filtrado = x_pred[0, 0]
        theta_filtrado = (theta_filtrado + np.pi) % (2 * np.pi) - np.pi
        
        return float(theta_filtrado)
