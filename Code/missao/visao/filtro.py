import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

class FiltroAlvoEKF:
    def __init__(self, dt_inicial=0.1):
        """
        Inicializa o Filtro de Kalman Estendido (EKF).
        Estado X: [x, y, vx, vy]
        Medição Z: [x, y]
        """
        # dim_x: Número de variáveis de estado
        # dim_z: Número de variáveis de medição
        self.ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)
        
        # Estado Inicial
        self.ekf.x = np.array([0.0, 0.0, 0.0, 0.0])
        
        # Matriz de Covariância de Estado (P) - Quão incertos estamos do estado inicial
        self.ekf.P *= 1000.0
        
        # Matriz de Ruído de Medição (R) - Quão barulhenta é a YOLO
        # Aumente esses valores se a YOLO pular muito
        ruido_yolo = 50.0 
        self.ekf.R = np.array([[ruido_yolo, 0.0],
                               [0.0, ruido_yolo]])
                               
        # Matriz de Ruído do Processo (Q) - Quão rápido a velocidade da armadilha/drone pode mudar
        self.ekf.Q = np.eye(4) * 0.1 
        
        self.iniciado = False
        self.dt_anterior = dt_inicial

    def H_jacobian(self, x):
        """
        Calcula a Jacobiana da função de observação h(x).
        Como observamos apenas x e y (índices 0 e 1 do estado), a derivada parcial
        em relação a x é 1, em relação a y é 1, e 0 para as velocidades.
        (Linear no momento, mas pronta para fórmulas não-lineares de PBVS)
        """
        return np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])

    def h_observacao(self, x):
        """
        Função não-linear h(x) que mapeia o estado atual para o que esperamos medir.
        No futuro (PBVS), aqui você aplicará as transformações de câmera (ex: X_cam = K * X_mundo).
        """
        return np.array([x[0], x[1]])

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
                self.ekf.x = np.array([z_x, z_y, 0.0, 0.0])
                self.iniciado = True
                return z_x, z_y
            else:
                return None, None # Ainda não temos nada para iniciar
            
        # --- PASSO 1: PREDIÇÃO ---
        # Matriz Jacobiana de Transição de Estado (F)
        self.ekf.F = np.array([
            [1.0, 0.0,  dt, 0.0],
            [0.0, 1.0, 0.0,  dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        
        self.ekf.predict()
        
        # --- PASSO 2: ATUALIZAÇÃO (com a medição da YOLO) ---
        # Se a IA não detectou a gaiola neste frame, pulamos a atualização
        # e confiamos apenas na PREDIÇÃO do filtro para manter o drone seguindo o alvo
        if z_x is not None and z_y is not None:
            z = np.array([z_x, z_y])
            self.ekf.update(z, HJacobian=self.H_jacobian, Hx=self.h_observacao)
        
        self.dt_anterior = dt
        
        # Retorna apenas as coordenadas x e y limpas
        x_limpo = self.ekf.x[0]
        y_limpo = self.ekf.x[1]
        
        return x_limpo, y_limpo
