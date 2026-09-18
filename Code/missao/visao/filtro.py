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
        self.timeout_limite = 1.5  # Segundos

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
