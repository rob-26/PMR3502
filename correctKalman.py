import numpy as np
import scipy as sp
import math

class KalmanFilter(object):

    ## Deve ser passado o valor de sigma_omega**2 na incerteza
    def __init__(self, callback, t0, rumo0, incerteza0):

        self._callback = callback

        # Estado inicial: [theta, omega]
        self._mu = np.array([rumo0, 0.0])  # rumo inicial, velocidade angular 0
        self._sigma = np.array([[incerteza0, 0.0], [0.0, 0.0]])  # incerteza no rumo apenas; incerteza0 já está ao quadrado
        self._t = t0  # instante inicial

        # Parâmetros do filtro    refazer os comentários
        self._sigma_omega = incerteza0  # variância do giroscópio (rad²/s²)
        self._sigma_theta = 0.05  # variância do magnetômetro (rad²)
        self._Q = self._sigma_theta  # ruído de medida do magnetômetro    ### renomear

        ## Etapa de predição 
        # Aqui precisamos das matrizes A, B, R; trabalhamos com o estado atual e a entrada de controle

    def predict(self, dt, vel_ang): #CHECK

        # Matriz de dinâmica
        A = np.array([[1, dt/2.0],
                        [0.0, 0.0]])
        
        # Matriz de atuação
        B = np.array([[dt/2.0],
                        [1.0]])

        # Matriz de covariância
        R = self._sigma_omega * np.array([[dt**2/4.0, dt/2.0],
                                            [dt/2.0, 1.0]])

        # Entrada de controle
        u = np.array([vel_ang])
        
        self._mu = A @ self._mu + B @ u
        self._sigma = A @ self._sigma @ A.T + R
    
        return self._mu
    
    def angle_diff(self, a, b):
        
        d = a - b
        normalized = (d + np.pi) % (2 * np.pi) - np.pi
        return normalized
    
    def update(self, ang_rumo):

        C = np.array([1.0, 0.0]).reshape((1,2))

        S = C @ self._sigma @ C.T + self._Q

        K = self._sigma @ C.T @ np.linalg.inv(S)

        theta_predicted = C @ self._mu.T

        obser_diff = self.angle_diff(ang_rumo, theta_predicted)

        self._mu = self._mu + K @ obser_diff
        self._sigma = (np.identity(2) - K @ C) @ self._sigma
        
        return self._mu


KF = KalmanFilter(100, 0, 0, 0.0386)

theta = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0])
omega = np.array([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])

for i in range(16):
    
    x_pred, y_pred = KF.predict(1, omega[i])
    x_up, y_up = KF.update(theta[i])



    print("Valor previsto:" , x_pred, y_pred)
    print("Valor intermediário:", x_up, y_up)









