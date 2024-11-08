from all_models import ModelGeo
import numpy as np

class Param:
    def __init__(self, N_horizon=20, tf=0.2, Tsim=0.01, m0=0.445, m1=1.2, m2=1.2,
                 J0=0.15, g=9.81, l1=1, l2=1, rho1=1, rho2=1, 
                 e2=np.array([0,1,0]), e3=np.array([0,0,1])):
        self.N_horizon = N_horizon
        self.tf = tf
        self.Tsim = Tsim
        self.m0 = m0
        self.m1 = m1
        self.m2 = m2
        self.J0 = J0
        self.g = g
        self.l1 = l1
        self.l2 = l2
        self.rho1 = rho1
        self.rho2 = rho2
        self.e2 = e2
        self.e3 = e3
 
# 使用结构体
param = Param()
geo = ModelGeo(param)