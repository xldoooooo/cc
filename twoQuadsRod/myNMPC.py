import os
import sys
import shutil
import errno
import timeit

from acados_template import AcadosOcp,AcadosOcpSolver,AcadosSim,AcadosSimSolver

import casadi as ca
import numpy as np
import scipy.linalg

from my_Model import myModel
from model_dae import ModelDae

class myNMPC(object):
    def __init__(self, myModel, tf):
        model = myModel
        self.tf = tf
        self.N_horizon = int(tf/0.01)

        nx = model.x.size1()
        self.nx = nx
        nu = model.u.size1()
        self.nu = nu
        
        acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        
        # create ocp
        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        ocp.model = model

        # set prediction horizon
        ocp.solver_options.N_horizon = self.N_horizon
        ocp.solver_options.tf = self.tf

        # set cost
        W_x = scipy.linalg.block_diag(np.eye(6),np.eye(6),4*np.eye(12))
        W_u = scipy.linalg.block_diag(np.eye(6),np.eye(2))
        m0 = 0.445
        g = 9.81
        u0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m0*g/2, m0*g/2])
        # initial cost term
        ocp.cost.cost_type_0 = 'NONLINEAR_LS'
        ocp.cost.W_0 = W_u
        ocp.cost.yref_0 = np.zeros(nu)
        ocp.model.cost_y_expr_0 = model.u-u0
        # path cost term
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.W = scipy.linalg.block_diag(W_x, W_u)
        ocp.cost.yref = np.zeros(nx+nu)
        e2 = np.array([0,1,0])
        e3 = np.array([0,0,1])
        ocp.model.cost_y_expr = ca.vertcat(
            model.x[0:3]-np.array([10,0,0]),
            ca.cross(model.x[3:6],-e2),
            ca.cross(model.x[6:9],e3),
            ca.cross(model.x[9:12],e3),
            model.x[12:15],
            model.x[15:18]+hat(model.x[3:6])@hat(model.x[3:6])@(-e2),
            model.x[18:21]+hat(model.x[6:9])@hat(model.x[6:9])@(e3),
            model.x[21:24]+hat(model.x[9:12])@hat(model.x[9:12])@(e3),
            model.u-u0
        )
        # terminal cost term
        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.cost.W_e = W_x
        ocp.cost.yref_e = np.zeros(nx)
        ocp.model.cost_y_expr_e = ca.vertcat(
            model.x[0:3]-np.array([10,0,0]),
            ca.cross(model.x[3:6],-e2),
            ca.cross(model.x[6:9],e3),
            ca.cross(model.x[9:12],e3),
            model.x[12:15],
            model.x[15:18]+hat(model.x[3:6])@hat(model.x[3:6])@(-e2),
            model.x[18:21]+hat(model.x[6:9])@hat(model.x[6:9])@(e3),
            model.x[21:24]+hat(model.x[9:12])@hat(model.x[9:12])@(e3)
        )

        # set constraints
        ocp.constraints.x0 = np.concatenate((np.zeros(3), -e2, e3, e3, np.zeros(12)))

        # set options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
        # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
        # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = 'ERK'
        # ocp.solver_options.print_level = 1
        ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP
        ocp.solver_options.globalization = 'MERIT_BACKTRACKING' # turns on globalization

        self.solver = AcadosOcpSolver(ocp)

        # define simulation
        self.integrator = AcadosSimSolver(ocp)
        

    def simulation(self, x0, xs, N_sim):
        N = int(N_sim/0.01)
        simX = np.zeros((N+1, self.nx))
        simU = np.zeros((N, self.nu))
        x_current = x0
        simX[0,:] = x0
        time_record = np.zeros(N)

        # closed loop

        for i in range(N):

            # solve ocp
            start = timeit.default_timer()
            self.solver.set(0, 'lbx', x_current)
            self.solver.set(0, 'ubx', x_current)
            nmpc_status = self.solver.solve()
            if nmpc_status != 0:
                raise Exception('acados acados_ocp_solver returned status {}. Exiting.'.format(nmpc_status))
            
            simU[i,:] = self.solver.get(0,'u')
            time_record[i] = timeit.default_timer() - start

            # simulate system
            self.integrator.set('x', x_current)
            self.integrator.set('u',simU[i,:])
            sim_status = self.integrator.solve()
            if sim_status != 0:
                raise Exception('acados integrator returned status {}. Exiting.'.format(sim_status))
            
            # update state
            x_current = self.integrator.get('x')
            simX[i+1,:] = x_current

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
    param.m0 = 0.445

    nmpc_model = myModel()
    ocp = myNMPC(myModel=nmpc_model.model, myConstraint=nmpc_model.constraint, tf=0.5)
    e2 = np.array([0,1,0])
    e3 = np.array([0,0,1])
    x0 = np.concatenate((np.zeros(3), -e2, e3, e3, np.zeros(12)))
    ocp.simulation(x0, x0, 20)



