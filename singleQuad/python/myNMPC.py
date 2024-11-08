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

class myNMPC(object):
    def __init__(self, myModel, myConstraint, tf):
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

        # param
        param = ca.SX.sym('param',18)
        ocp.model.p = param
        ocp.parameter_values = np.zeros(18)

        # set cost
        x_ref = param[0:6]
        u_ref = param[6:9]
        W_x = ca.diag(param[9:15])
        W_u = ca.diag(param[15:18])
        # initial cost term
        ocp.cost.cost_type_0 = 'EXTERNAL'
        ocp.model.cost_expr_ext_cost_0 = (model.u-u_ref).T @ W_u @ (model.u-u_ref) 

        # path cost term
        ocp.cost.cost_type = 'EXTERNAL'
        W = ca.diagcat(W_x, W_u)
        ocp.model.cost_expr_ext_cost = ca.vertcat(
            model.x[0:6]-x_ref,
            model.u-u_ref).T @ W @ ca.vertcat(
            model.x[0:6]-x_ref,
            model.u-u_ref)
        # terminal cost term
        ocp.cost.cost_type_e = 'EXTERNAL'
        ocp.model.cost_expr_ext_cost_e = ca.vertcat(
            model.x[0:6]-x_ref).T @ W_x @ ca.vertcat(
            model.x[0:6]-x_ref)
        
        # set constraints
        ocp.constraints.x0 = np.zeros(nx)
        ocp.constraints.lbu = np.zeros(nu)
        ocp.constraints.ubu = np.zeros(nu)
        ocp.constraints.idxbu = np.array([1,1,1])

        # set options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
        # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
        # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
        ocp.solver_options.hessian_approx = 'EXACT' # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = 'ERK'
        # ocp.solver_options.print_level = 1
        ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP
        ocp.solver_options.globalization = 'MERIT_BACKTRACKING' # turns on globalization

        self.solver = AcadosOcpSolver(ocp)

        # define simulation
        self.integrator = AcadosSimSolver(ocp)
        

    def simulation(self, x0, N_sim):
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
            lbu = np.array([-25,-25,-25])
            ubu = np.array([25,25,25])
            for j in range(self.N_horizon):
                self.solver.set(j,'lbu', lbu)
                self.solver.set(j,'ubu', ubu)
            param = np.zeros(18)
            param[9:18] = np.ones(9)
            m1 = 0.98
            g = 9.81
            e3 = np.array([0, 0, 1])
            param[6:9] = -m1*g*e3
            for k in range(self.N_horizon+1):
                param[0] = i*0.01*0.5 + k*0.01*0.5
                self.solver.set(k,'p',param)
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
            # print(x_current)
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
    nmpc_model = myModel()
    ocp = myNMPC(myModel=nmpc_model.model, myConstraint=nmpc_model.constraint, tf=0.2)
    x0 = np.concatenate((np.zeros(3), np.zeros(3)))
    ocp.simulation(x0, 20)



