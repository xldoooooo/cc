import os
import sys
import shutil
import errno
import timeit

import casadi as ca
import numpy as np
import scipy

from all_models import ModelSihao
from all_models import ModelDae
from all_models import ModelGeo

from setMhe import setMhe
from setOcp import setOcp
from setSim import setSim

class Param:
    def __init__(self, N_mhe=20, N_ocp=10, Tsim=0.01, m0=0.445, m1=1.2, m2=1.2,
                 J0=0.15, g=9.81, l1=1, l2=1, rho1=1, rho2=1, 
                 e2=np.array([0,1,0]), e3=np.array([0,0,1])):
        self.N_mhe = N_mhe
        self.N_ocp = N_ocp
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

class main():
    def __init__(self, model_mhe, model_ocp, model_sim, param):
        self.mhe_solver = setMhe(model_mhe,param)
        self.ocp_solver = setOcp(model_ocp,param)
        self.sim_solver = setSim(model_sim,param)

        self.nx = model_sim.model.x.size1()
        self.nu = model_sim.model.u.size1()
        self.x0_sim = model_sim.x0_sim
        self.x0hat_sim = model_sim.x0hat_sim
        self.u0_sim = model_sim.u0_sim
        self.u0_ocp = model_ocp.u0_ocp
        

    def simulation(self, N_sim, param):
        Tsim = param.Tsim
        N_mhe = param.N_mhe
        m0 = param.m0
        m1 = param.m1
        m2 = param.m2
        rho1 = param.rho1
        rho2 = param.rho2
        l1 = param.l1
        l2 = param.l2
        J0 = param.J0
        x0_sim = self.x0_sim
        x0hat_sim = self.x0hat_sim
        u0_sim = self.u0_sim
        u0_ocp = self.u0_ocp

        N = int(N_sim/Tsim)
        simX = np.zeros((N+1, self.nx))
        simXhat = np.zeros((N+1, self.nx))
        simU = np.zeros((N, self.nu))
        simY = np.zeros((N+1, 12))
        simX[0,:] = x0_sim
        simXhat[0,:] = x0hat_sim
        simU[0,:] = u0_sim
        time_record = np.zeros(N)

        # closed loop
        for i in range(N):
            start = timeit.default_timer()
            x_current = simX[i,:]
            y_current = np.concatenate((x_current[6:9],
                                        x_current[9:12],
                                        x_current[18:21],
                                        x_current[21:24]))
            simY[i,:] = y_current
            for j in range(N_mhe):
                self.mhe_solver.set(j,'x',x0hat_sim)
                if i-N_mhe+j >= 0:
                    self.mhe_solver.set(j,'y_ref',np.hstack((simY[i-N_mhe+j,:],simU[i-N_mhe+j,:])))
                else:
                    self.mhe_solver.set(j,'y_ref',np.hstack((simY[0,:],simU[0,:])))  
            status = self.mhe_solver.solve()
            if status != 0:
                raise Exception('acados acados_mhe_solver returned status {}. Exiting.'.format(status))
            x0hat_sim = self.mhe_solver.get(N_mhe,'x')
            # x0hat_sim = x_current
            x0_ocp = x0hat_sim
            q0 = x0_ocp[3:6]
            q1 = x0_ocp[6:9]
            q2 = x0_ocp[9:12]
            omega0 = x0_ocp[15:18]
            omega1 = x0_ocp[18:21]
            omega2 = x0_ocp[21:24]
            self.ocp_solver.set(0,'lbx',x0_ocp)
            self.ocp_solver.set(0,'ubx',x0_ocp)
            status = self.ocp_solver.solve()
            if status != 0:
                raise Exception('acados acados_ocp_solver returned status {}. Exiting.'.format(status))
            u0_ocp = self.ocp_solver.get(0,'u')
            domega1 = u0_ocp[0:3]
            domega2 = u0_ocp[3:6]
            T1 = u0_ocp[6]
            T2 = u0_ocp[7]
            domega0 = (-rho1*T1*np.cross(q0,q1)+rho2*T2*np.cross(q0,q2))/J0
            u1 = -m1/m0*(T1*q1+T2*q2)-T1*q1+m1*rho1*(np.cross(domega0,q0)+hat(omega0)**2@q0)-m1*l1*(np.cross(domega1,q1)-np.linalg.norm(omega1)**2*q1)
            u2 = -m2/m0*(T1*q1+T2*q2)-T2*q2-m2*rho2*(np.cross(domega0,q0)+hat(omega0)**2@q0)-m2*l2*(np.cross(domega2,q2)-np.linalg.norm(omega2)**2*q2)
            u0_sim = np.concatenate((u1,u2))
            simU[i,:] = u0_sim.T
            time_record[i] = timeit.default_timer() - start

            # simulate system
            self.sim_solver.set('x', x_current)
            self.sim_solver.set('u', u0_sim)
            status = self.sim_solver.solve()
            if status != 0:
                raise Exception('acados integrator returned status {}. Exiting.'.format(status))
            
            # update state
            simX[i+1,:] = self.sim_solver.get('x')

        print("average estimation time is {}".format(time_record.mean()))
        print("max estimation time is {}".format(time_record.max()))
        print("min estimation time is {}".format(time_record.min()))
        print("final state is {}".format(x_current))


def hat(a):
    hat_a = ca.blockcat([
        [0,  -a[2],  a[1]],
        [a[2],  0,  -a[0]],
        [-a[1],  a[0],  0]])
    return hat_a 




if __name__ == '__main__':
    param1 = Param()
    sihao = ModelSihao(param1)
    dae = ModelDae(param1)
    geo = ModelGeo(param1)
    kk = main(model_mhe=geo, model_ocp=sihao, model_sim=dae, param=param1)
    kk.simulation(20, param1)



