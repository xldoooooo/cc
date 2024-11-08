import numpy as np
import casadi as ca
import scipy.linalg
from acados_template import AcadosModel

class ModelGeo():
    def __init__(self,param):
        model = AcadosModel()
        m0 = param.m0
        m1 = param.m1
        m2 = param.m2
        mT = m0+m1+m2
        g = param.g
        l1 = param.l1
        l2 = param.l2
        rho1 = param.rho1
        rho2 = param.rho2
        J0 = param.J0
        J0_bar = J0 + m1*rho1**2 + m2*rho2**2
        e3 = param.e3
        
        u1 = ca.SX.sym('u1',3)
        u2 = ca.SX.sym('u2',3)
        u = ca.vertcat(u1,u2)   
        p0 = ca.SX.sym('p0',3)
        q0 = ca.SX.sym('q0',3)
        q1 = ca.SX.sym('q1',3)
        q2 = ca.SX.sym('q2',3)
        dp0 = ca.SX.sym('dp0',3)
        omega0 = ca.SX.sym('omega0',3)
        omega1 = ca.SX.sym('omega1',3)
        omega2 = ca.SX.sym('omega2',3)
        dq0 = ca.cross(omega0,q0)
        dq1 = ca.cross(omega1,q1)
        dq2 = ca.cross(omega2,q2)
        x = ca.vertcat(p0,q0,q1,q2,dp0,omega0,omega1,omega2)
        xdot = ca.SX.sym('xdot', x.size1())
        M = ca.vertcat(ca.horzcat(mT*np.eye(3),(-m1*rho1+m2*rho2)*hat(q0),m1*l1*hat(q1),m2*l2*hat(q2)),
                       ca.horzcat((m1*rho1-m2*rho2)*hat(q0),J0_bar*np.eye(3),m1*rho1*l1*hat(q0)@hat(q1),-m2*rho2*l2*hat(q0)@hat(q2)),
                       ca.horzcat(-m1*hat(q1),m1*rho1*hat(q1)@hat(q0),m1*l1*np.eye(3),np.zeros((3,3))),
                       ca.horzcat(-m2*hat(q2),-m2*rho2*hat(q2)@hat(q0),np.zeros((3,3)),m2*l2*np.eye(3)))
        F = ca.vertcat(-m1*l1*ca.norm_2(omega1)**2*q1+m1*rho1*ca.norm_2(omega0)**2*q0-m2*l2*ca.norm_2(omega2)**2*q2-m2*rho2*ca.norm_2(omega0)**2*q0+u1+u2+mT*g*e3,
                       rho1*hat(q0)@(m1*g*e3-m1*l1*ca.norm_2(omega1)**2*q1+u1)-rho2*hat(q0)@(m2*g*e3-m2*l2*ca.norm_2(omega2)**2*q2+u2),
                       -hat(q1)@(m1*g*e3+m1*rho1*ca.norm_2(omega0)**2*q0+u1),
                       -hat(q2)@(m2*g*e3-m2*rho2*ca.norm_2(omega0)**2*q0+u2))
        f_expl_expr = ca.vertcat(dp0,
                                 dq0,
                                 dq1,
                                 dq2,
                                 ca.solve(M,F))
        f_impl_expr = xdot - f_expl_expr
        model.f_expl_expr = f_expl_expr
        model.f_impl_expr = f_impl_expr
        model.x = x
        model.u = u
        model.p = []
        model.xdot = xdot
        model.name = 'geo'
        self.model = model
        self.integrator_type = 'ERK'
        self.cost_y_expr_0 = ca.vertcat(p0+rho1*q0-l1*q1,
                                        p0-rho2*q0-l2*q2,
                                        dp0+rho1*dq0-l1*dq1,
                                        dp0-rho2*dq0-l2*dq2,
                                        u)
        self.cost_y_expr = ca.vertcat(p0+rho1*q0-l1*q1,
                                      p0-rho2*q0-l2*q2,
                                      dp0+rho1*dq0-l1*dq1,
                                      dp0-rho2*dq0-l2*dq2,
                                      u)
        self.Wy_mhe = np.eye(12)
        self.Wu_mhe = np.eye(u.size1())
        self.y0_mhe = np.concatenate((np.array([0,-1,-1]),
                                      np.array([0,1,-1]),
                                      np.zeros(6)))
        self.u0_mhe = np.concatenate((-m1*g*e3-0.5*m0*g*e3,
                                      -m2*g*e3-0.5*m0*g*e3))
        
        # self.con_h_expr_0 = ca.vertcat(1-ca.norm_2(q0)**2,
        #                                ca.dot(q0,omega0),
        #                                1-ca.norm_2(q1)**2,
        #                                ca.dot(q1,omega1),
        #                                1-ca.norm_2(q2)**2,
        #                                ca.dot(q2,omega2))
        # self.con_h_expr = ca.vertcat(1-ca.norm_2(q0)**2,
        #                              ca.dot(q0,omega0),
        #                              1-ca.norm_2(q1)**2,
        #                              ca.dot(q1,omega1),
        #                              1-ca.norm_2(q2)**2,
        #                              ca.dot(q2,omega2))

class ModelDae():
    def __init__(self,param):
        model = AcadosModel()
        m0 = param.m0
        m1 = param.m1
        m2 = param.m2
        g = param.g
        l1 = param.l1
        l2 = param.l2
        rho1 = param.rho1
        rho2 = param.rho2
        J0 = param.J0
        e3 = param.e3
        e2 = param.e2

        u1 = ca.SX.sym('u1',3)
        u2 = ca.SX.sym('u2',3)
        u = ca.vertcat(u1,u2)   
        p0 = ca.SX.sym('p0',3)
        q0 = ca.SX.sym('q0',3)
        p1 = ca.SX.sym('p1',3)
        p2 = ca.SX.sym('p2',3)
        dp0 = ca.SX.sym('dp0',3)
        omega0 = ca.SX.sym('omega0',3)
        dp1 = ca.SX.sym('dp1',3)
        dp2 = ca.SX.sym('dp2',3)
        dq0 = ca.cross(omega0,q0)
        x = ca.vertcat(p0,q0,p1,p2,dp0,omega0,dp1,dp2)
        xdot = ca.SX.sym('xdot', x.size1())
        z1 = ca.SX.sym('z1',1)
        z2 = ca.SX.sym('z2',1)
        z = ca.vertcat(z1,z2)

        c1 = 1/2*(ca.norm_2(p0+rho1*q0-p1)**2-l1**2)
        c2 = 1/2*(ca.norm_2(p0-rho2*q0-p2)**2-l2**2)
        c = ca.vertcat(c1,c2)
        D_p0_c1 = p0 + rho1*q0 - p1
        D_p0_c2 = p0 - rho2*q0 - p2
        D_q0_c1 = rho1*(p0 + rho1*q0 - p1)
        D_q0_c2 = -rho2*(p0 - rho2*q0 - p2)
        D_p1_c1 = -(p0 + rho1*q0 - p1)
        D_p2_c2 = -(p0 - rho2*q0 - p2)
        DD_p0_c1 = dp0 + rho1*dq0 - dp1
        DD_p0_c2 = dp0 - rho2*dq0 - dp2
        DD_q0_c1 = rho1*(dp0 + rho1*dq0 - dp1)
        DD_q0_c2 = -rho2*(dp0 - rho2*dq0 - dp2)
        DD_p1_c1 = -(dp0 + rho1*dq0 - dp1)
        DD_p2_c2 = -(dp0 - rho2*dq0 - dp2)
        dc1 = ca.dot(D_p0_c1,dp0) + ca.dot(D_q0_c1,dq0) + ca.dot(D_p1_c1,dp1)
        dc2 = ca.dot(D_p0_c2,dp0) + ca.dot(D_q0_c2,dq0) + ca.dot(D_p2_c2,dp2)
        dc = ca.vertcat(dc1,dc2)
        M11 = ca.diagcat(m0*np.eye(3),J0*np.eye(3),m1*np.eye(3),m2*np.eye(3))
        M12 = ca.blockcat([[D_p0_c1, D_p0_c2],
                           [hat(q0)@D_q0_c1, hat(q0)@D_q0_c2],
                           [D_p1_c1, np.zeros(3)],
                           [np.zeros(3), D_p2_c2]])
        M21 = ca.transpose(M12)
        M22 = np.zeros((2,2))
        M = ca.blockcat([[M11, M12],
                         [M21, M22]]) 
        F = ca.vertcat(-ca.dot(DD_p0_c1,dp0)+ca.norm_2(omega0)**2*ca.dot(D_q0_c1,q0)-ca.dot(DD_q0_c1,dq0)-ca.dot(DD_p1_c1,dp1),
                       -ca.dot(DD_p0_c2,dp0)+ca.norm_2(omega0)**2*ca.dot(D_q0_c2,q0)-ca.dot(DD_q0_c2,dq0)-ca.dot(DD_p2_c2,dp2))
        p = 5
        FF = ca.vertcat(m0*g*e3,
                        np.zeros(3),
                        m1*g*e3+u1,
                        m2*g*e3+u2,
                        F-p**2*c-2*p*dc)
        f_impl_expr = ca.vertcat(xdot[0:3]-dp0,
                                 xdot[3:6]-dq0,
                                 xdot[6:9]-dp1,
                                 xdot[9:12]-dp2,
                                 M@ca.vertcat(xdot[12:24],z)-FF)
        model.f_impl_expr = f_impl_expr

        model.u = u
        model.x = x
        model.z = z
        model.xdot = xdot
        model.name = 'dae'

        self.model = model
        self.integrator_type = 'IRK'
        self.Wy_mhe = np.eye(12)
        self.Wu_mhe = np.eye(u.size1())

        # self.con_h_expr_0 = ca.vertcat(1-ca.norm_2(q0)**2,ca.dot(q0,omega0))
        # self.con_h_expr = ca.vertcat(1-ca.norm_2(q0)**2,ca.dot(q0,omega0))
        # self.con_h_expr_e = ca.vertcat(1-ca.norm_2(q0)**2,ca.dot(q0,omega0),c,dc)
        self.cost_y_expr_0 = ca.vertcat(p1,p2,dp1,dp2,u)
        self.cost_y_expr = ca.vertcat(p1,p2,dp1,dp2,u)
        self.x0_sim = np.concatenate((np.zeros(3),
                                      -e2,
                                      np.array([0,-1,-1]),
                                      np.array([0,1,-1]),
                                      np.zeros(12)))
        self.x0hat_sim = np.concatenate((np.zeros(3),
                                         -e2,
                                         e3,
                                         e3,
                                         np.zeros(12)))
        self.u0_sim = np.concatenate((-m1*g*e3-0.5*m0*g*e3,
                                      -m2*g*e3-0.5*m0*g*e3))
        self.y0_mhe = np.concatenate((np.array([0,-1,-1]),
                                      np.array([0,1,-1]),
                                      np.zeros(6)))
        self.u0_mhe = np.concatenate((-m1*g*e3-0.5*m0*g*e3,
                                      -m2*g*e3-0.5*m0*g*e3))

class ModelSihao():
    def __init__(self,param):
        model = AcadosModel()
        m0 = param.m0
        m1 = param.m1
        m2 = param.m2
        g = param.g
        l1 = param.l1
        l2 = param.l2
        rho1 = param.rho1
        rho2 = param.rho2
        J0 = param.J0
        e2 = param.e2
        e3 = param.e3

        domega1 = ca.SX.sym('domega1',3)
        domega2 = ca.SX.sym('domega2',3)
        T1 = ca.SX.sym('T1')
        T2 = ca.SX.sym('T2')
        u = ca.vertcat(domega1,domega2,T1,T2)   
        p0 = ca.SX.sym('p0',3)
        q0 = ca.SX.sym('q0',3)
        q1 = ca.SX.sym('q1',3)
        q2 = ca.SX.sym('q2',3)
        dp0 = ca.SX.sym('dp0',3)
        omega0 = ca.SX.sym('omega0',3)
        omega1 = ca.SX.sym('omega1',3)
        omega2 = ca.SX.sym('omega2',3)
        dq0 = ca.cross(omega0,q0)
        dq1 = ca.cross(omega1,q1)
        dq2 = ca.cross(omega2,q2)
        x = ca.vertcat(p0,q0,q1,q2,dp0,omega0,omega1,omega2)

        xdot = ca.SX.sym('xdot', x.size1())

        f_expl_expr = ca.vertcat(dp0,
                                 dq0,
                                 dq1,
                                 dq2,
                                 (-T1*q1-T2*q2)/m0+g*e3,
                                 (-rho1*T1*ca.cross(q0,q1)+rho2*T2*ca.cross(q0,q2))/J0,
                                 domega1,
                                 domega2)
        model.f_expl_expr = f_expl_expr

        model.u = u
        model.x = x
        model.xdot = xdot
        model.name = 'sihao'

        self.model = model
        self.integrator_type = 'ERK'
        self.Wx_ocp = 3*scipy.linalg.block_diag(np.eye(6),np.eye(6),np.eye(12))
        self.Wu_ocp = np.eye(u.size1())

        u0 = ca.vertcat(np.zeros(6),0.5*m0*g,0.5*m0*g)
        self.cost_y_expr_0 = u-u0
        self.cost_y_expr = ca.vertcat(p0-np.array([20,0,0]),
                                       ca.cross(q0,-e2),
                                       ca.cross(q1,e3),
                                       ca.cross(q2,e3),
                                       dp0,
                                       omega0+hat(q0)**2@(-e2),
                                       omega1+hat(q1)**2@(e3),
                                       omega2+hat(q2)**2@(e3),
                                       u-u0)
        self.cost_y_expr_e = ca.vertcat(p0-np.array([20,0,0]),
                                       ca.cross(q0,-e2),
                                       ca.cross(q1,e3),
                                       ca.cross(q2,e3),
                                       dp0,
                                       omega0+hat(q0)**2@(-e2),
                                       omega1+hat(q1)**2@(e3),
                                       omega2+hat(q2)**2@(e3))
        self.u0_ocp = u0

def hat(a):
    hat_a = ca.blockcat([
        [0,  -a[2],  a[1]],
        [a[2],  0,  -a[0]],
        [-a[1],  a[0],  0]])
    return hat_a



