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
        # Agora o erro é em METROS FÍSICOS. Uma variação de YOLO costuma ser de ~5cm (0.05m)
        ruido_yolo = 0.05 
        self.kf.R = np.array([[ruido_yolo, 0.0],
                              [0.0, ruido_yolo]])
                               
        # Matriz de Ruído do Processo (Q) - Quão rápido a velocidade da armadilha/drone pode mudar
        # Ajustado para escala em metros (m/s).
        self.kf.Q = np.eye(4) * 0.5 
        
        # Matriz de Observação (H) - Como extraímos a medição Z a partir do estado X
        self.kf.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])
        
        self.iniciado = False
        self.dt_anterior = dt_inicial

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
        
        self.dt_anterior = dt
        
        # Retorna apenas as coordenadas x e y limpas
        x_limpo = float(self.kf.x[0, 0])
        y_limpo = float(self.kf.x[1, 0])
        
        return x_limpo, y_limpo
