import numpy as np
import scipy as sp
import math


def cria_estimador_rumo(t0, rumo0, incerteza0, callback, nome_estimador = "bussola"):
    
    if nome_estimador=="bussola":
        return Bussola(callback, t0, rumo0, incerteza0)

    elif nome_estimador == "kf":
        return KalmanFilter (callback, t0, rumo0, incerteza0)

    else:
        raise ValueError(f"Estimador {nome_estimador} desconhecido!")

class Bussola():
    def __init__(self, callback, t0, rumo0, incerteza0):
        self._angulo = rumo0
        self._t = t0
        self._nova_previsao = callback
        pass

    def processa_dados(self, t, mx, my, mz):
         # t é o timestamp em picossegundos,
         # mx, my, mz são as coletas "brutas" do magnetômetro
         # Calcule o tempo e estimativa de rumo (em radianos!)
         # Notifique a nova estimativa com:
         # self._nova_previsao(tempo, angulo, 0, 0)
         pass

    def callback(self, data):
        wx = int.from_bytes(data[8:10], byteorder='big', signed=True)
        wy = int.from_bytes(data[10:12], byteorder='big', signed=True)
        wz = int.from_bytes(data[12:14], byteorder='big', signed=True)
        mx = int.from_bytes(data[14:16], byteorder='big', signed=True)
        my = int.from_bytes(data[16:18], byteorder='big', signed=True)
        mz = int.from_bytes(data[18:20], byteorder='big', signed=True)
        t = int.from_bytes(data[-8:], byteorder='little', signed=False)
        self.processa_dados(t, mx, my, mz)


class KalmanFilter(object):

    ## Deve ser passado o valor de sigma_omega**2 na incerteza
    def __init__(self, callback, t0, rumo0, incerteza0):

        self._callback = callback

        # Estado inicial: [theta, omega]
        self._mu = np.array([rumo0, 0.0])  # rumo inicial, velocidade angular 0
        self._sigma = np.array([[incerteza0, 0.0], [0.0, 0.0]])  # incerteza no rumo apenas; incerteza0 já está ao quadrado
        self._t = t0  # instante inicial

        # Parâmetros do filtro    refazer os comentários
        self._sigma_omega = incerteza0  
        self._sigma_theta = 0.05  
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


