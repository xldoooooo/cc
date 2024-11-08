import os
from acados_template import AcadosOcp,AcadosOcpSolver
import scipy.linalg
import numpy as np

def setOcp(model_ocp,param):
    acados_source_path = os.environ['ACADOS_SOURCE_DIR']
    acados_include_path = acados_source_path + '/include'
    acados_lib_path = acados_source_path + '/lib'

    ocp = AcadosOcp()
    ocp.acados_include_path = acados_include_path
    ocp.acados_lib_path = acados_lib_path
    ocp.model = model_ocp.model
    ocp.cost.cost_type_0 = 'NONLINEAR_LS'
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    Wx_ocp = model_ocp.Wx_ocp
    Wu_ocp = model_ocp.Wu_ocp
    ocp.cost.W_0 = Wu_ocp
    ocp.cost.W = scipy.linalg.block_diag(Wx_ocp,Wu_ocp)
    ocp.cost.W_e = Wx_ocp

    ocp.model.cost_y_expr_0 = model_ocp.cost_y_expr_0
    ocp.model.cost_y_expr = model_ocp.cost_y_expr
    ocp.model.cost_y_expr_e = model_ocp.cost_y_expr_e

    ocp.cost.yref_0 = np.zeros(model_ocp.cost_y_expr_0.size1())
    ocp.cost.yref = np.zeros(model_ocp.cost_y_expr.size1())
    ocp.cost.yref_e = np.zeros(model_ocp.cost_y_expr_e.size1())

    ocp.constraints.x0 = np.zeros(24)

    ocp.solver_options.N_horizon = param.N_ocp
    ocp.solver_options.tf = param.N_ocp*param.Tsim
    ocp.solver_options.integrator_type = model_ocp.integrator_type
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'

    
    ocp_solver = AcadosOcpSolver(ocp)
    return ocp_solver
