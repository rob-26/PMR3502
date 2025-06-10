import numpy as np
import scipy as sp
import math

class KalmanFilter(object):
    def __init__(self, dt, cov_omega, cov_theta):
        """
          :param dt: time step (time for 1 cycle)

        """
        # Period  CHECK
        self.dt = dt
        self.cov_omega = cov_omega
        self.cov_theta = cov_theta

        # Initial state CHECK
        self.mu = np.array([[0],
                            [0]])
        # First state CHECK
        self.mu_til = np.array([[0],
                                [0]])
        # Second state CHECK 
        self.mu_barra = np.array([[0],
                                  [0]])

        # State Transition Matrix A   CHECK
        self.A = np.array([[1, self.dt],
                           [0, 1]])

        # Avaliar possíveis problemas de dimensão da matriz
        # Intermediate Measurement Mapping Matrix  C_int CHECK
        
        self.C_int = np.array([[0, 1]])
        
        # Measurement Mapping Matrix  C CHECK
        
        self.C = np.array([[1,0]])

        # Process Noise Covariance R    CHECK
        self.R = cov_omega**2 * np.array([[self.dt**2/4 , self.dt/2],
                                                 [self.dt/2, 1 ]])



        # Intermediate Measurement Noise Covariance Q_int NÃO CHECK – PEGAR VALORES PERY
        self.Q_int = self.cov_omega ** 2 

        # Measurement Noise Covariance Q NÃO CHECK – PEGAR VALORES PERY
        self.Q = self.cov_theta ** 2 


        # Initial Covariance Matrix sigma
        self.sigma = np.array([[90**2 , 0],
                               [0, 0]])
        
        # Initial First Covariance Matrix sigma
        self.sigma_til = np.array([[0, 0],
                                   [0, 0]])

        # Initial Second Covariance Matrix sigma
        self.sigma_barra = np.array([[0, 0],
                                     [0, 0]])

    def predict(self): #CHECK
        self.mu_til = self.A @ self.mu
        self.sigma_til = self.A @ self.sigma @ np.transpose(self.A) + self.R
        return self.mu_til[0:2]

    # Os dados cx e cy são passados como parâmetro quando se chama o método

    def intermediate_update(self, cx, cy, dt):
    #cx e cy devem ser substituídos pelos dados dos sensores, a medida da velocidade angular pelo giroscópio
    # Olhar o tamanho das matrizes para validar as equações
        self.dt = dt
        z = np.array([[cy]])
        S = self.C_int @ self.sigma_til @ np.transpose(self.C_int) + self.Q_int
        K = self.sigma_til @ np.transpose(self.C_int) @ np.linalg.inv(S)
        self.mu_barra = self.mu_til + K @ ( z - self.C_int @ self.mu_til)
        self.sigma_barra = (np.identity(2) - K @ self.C_int) @ self.sigma_til
        return self.mu_barra[0:2]
   
    def final_update(self, cx,cy, dt):
    #cx e cy devem ser substituídos pelos dados dos sensores, a medida do rumo pelo giroscópio
    # Olhar o tamanho das matrizes para validar as equações
        self.dt = dt
        z = np.array([[cx]])
        S = self.C @ self.sigma_barra @ np.transpose(self.C) + self.Q
        K = self.sigma_barra @ np.transpose(self.C) @ np.linalg.inv(S)
        self.mu = self.mu_barra + K @ ( z - self.C @ self.mu_barra)
        self.sigma = (np.identity(2) - K @ self.C) @ self.sigma_barra
        return self.mu[0:2]

KF = KalmanFilter(2, 5, 90)

theta = np.array([0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30])
omega = np.array([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])

for i in range(16):
    
    x_pred, y_pred = KF.predict()
    x_up1, y_up1 = KF.intermediate_update(theta[i],omega[i],2)
    x_up2, y_up2 = KF.final_update(theta[i],omega[i],2)
    print("Valor previsto:" , x_pred, y_pred)
    print("Valor intermediário:", x_up1, y_up1)
    print("Valor final:", x_up2, y_up2)