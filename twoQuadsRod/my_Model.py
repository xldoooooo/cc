import numpy as np
import casadi as ca
from acados_template import AcadosModel

class myModel(object):
    def __init__(self):
        model = AcadosModel()
        constraint = ca.types.SimpleNamespace()
        # system params
        m0 = 0.445
        g = 9.81
        e3 = np.array([0, 0, 1])
        rho1 = rho2 = 1
        J0 = 1/3*m0*rho1**2
        # control inputs
        domega1 = ca.SX.sym('domega1',3)
        domega2 = ca.SX.sym('domega2',3)
        T1 = ca.SX.sym('T1')
        T2 = ca.SX.sym('T2')
        controls = ca.vertcat(domega1, domega2, T1, T2)
        # model states
        p0 = ca.SX.sym('p0',3)
        q0 = ca.SX.sym('q0',3)
        q1 = ca.SX.sym('q1',3)
        q2 = ca.SX.sym('q2',3)
        dp0 = ca.SX.sym('dp0',3)
        omega0 = ca.SX.sym('omega0',3)
        omega1 = ca.SX.sym('omega1',3)
        omega2 = ca.SX.sym('omega2',3)
        states = ca.vertcat(p0,q0,q1,q2,dp0,omega0,omega1,omega2)

        rhs = ca.vertcat(
            dp0,
            hat(omega0) @ q0,
            hat(omega1) @ q1,
            hat(omega2) @ q2,
            -T1/m0*q1-T2/m0*q2+g*e3,
            -rho1*T1/J0*hat(q0)@q1+rho2*T2/J0*hat(q0)@q2,
            domega1,
            domega2
        )
        x_dot = ca.SX.sym('x_dot', rhs.size1())
        f_expl_expr = rhs
        f_impl_expr = x_dot - rhs

        model.f_expl_expr = f_expl_expr
        model.f_impl_expr = f_impl_expr
        model.x = states
        model.u = controls
        model.p = []
        model.name = 'NMPC_model'

        self.model = model
        self.constraint = constraint



def hat(a):
    hat_a = ca.blockcat([
        [0,  -a[2],  a[1]],
        [a[2],  0,  -a[0]],
        [-a[1],  a[0],  0]])
    return hat_a



