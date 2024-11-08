import numpy as np
import casadi as ca
from acados_template import AcadosModel

class myModel(object):
    def __init__(self):
        model = AcadosModel()
        constraint = ca.types.SimpleNamespace()
        # system params
        m1 = 0.98
        g = 9.81
        e3 = np.array([0, 0, 1])
        # control inputs
        u1 = ca.SX.sym('u1',3)
        controls = u1
        # model states
        p1 = ca.SX.sym('p1',3)
        dp1 = ca.SX.sym('dp1',3)
        states = ca.vertcat(p1,dp1)
        

        rhs = ca.vertcat(
            dp1,
            u1/m1 + g*e3
        )
        x_dot = ca.SX.sym('x_dot', rhs.size1())
        f_expl_expr = rhs
        f_impl_expr = x_dot - rhs

        model.f_expl_expr = f_expl_expr
        model.f_impl_expr = f_impl_expr
        model.x = states
        model.u = controls
        
        model.name = 'NMPC_model'

        self.model = model
        self.constraint = constraint



def hat(a):
    hat_a = ca.blockcat([
        [0,  -a[2],  a[1]],
        [a[2],  0,  -a[0]],
        [-a[1],  a[0],  0]])
    return hat_a



