import os
from acados_template import AcadosOcp,AcadosOcpSolver
import scipy.linalg
import numpy as np

def setMhe(model_mhe,param):
    acados_source_path = os.environ['ACADOS_SOURCE_DIR']
    acados_include_path = acados_source_path + '/include'
    acados_lib_path = acados_source_path + '/lib'
        
    mhe = AcadosOcp() # create mhe
    mhe.acados_include_path = acados_include_path
    mhe.acados_lib_path = acados_lib_path
    mhe.model = model_mhe.model   
    mhe.cost.cost_type_0 = 'NONLINEAR_LS'
    mhe.cost.cost_type = 'NONLINEAR_LS'
    Wy_mhe = model_mhe.Wy_mhe
    Wu_mhe = model_mhe.Wu_mhe
    mhe.cost.W_0 = scipy.linalg.block_diag(Wy_mhe,Wu_mhe)
    mhe.cost.W = scipy.linalg.block_diag(Wy_mhe,Wu_mhe)

    mhe.model.cost_y_expr_0 = model_mhe.cost_y_expr_0
    mhe.model.cost_y_expr = model_mhe.cost_y_expr
    mhe.cost.yref_0 = np.zeros(model_mhe.cost_y_expr_0.size1())
    mhe.cost.yref = np.zeros(model_mhe.cost_y_expr.size1())
    # mhe.cost.yref_e = np.zeros((0,))

    # mhe.model.con_h_expr_0 = model_mhe.con_h_expr_0
    # mhe.constraints.uh_0 = np.zeros(model_mhe.con_h_expr_0.size1())
    # mhe.constraints.lh_0 = np.zeros(model_mhe.con_h_expr_0.size1())
    # mhe.model.con_h_expr = model_mhe.con_h_expr
    # mhe.constraints.uh = np.zeros(model_mhe.con_h_expr.size1())
    # mhe.constraints.lh = np.zeros(model_mhe.con_h_expr.size1())
    # mhe.model.con_h_expr_e = model_mhe.con_h_expr_e
    # mhe.constraints.uh_e = np.zeros(model_mhe.con_h_expr_e.size1())
    # mhe.constraints.lh_e = np.zeros(model_mhe.con_h_expr_e.size1())
        
    mhe.solver_options.N_horizon = param.N_mhe
    mhe.solver_options.tf = param.N_mhe*param.Tsim
    mhe.solver_options.integrator_type = model_mhe.integrator_type
    mhe.solver_options.nlp_solver_type = 'SQP_RTI'
    mhe.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    mhe.solver_options.hessian_approx = 'GAUSS_NEWTON'

    mhe_solver = AcadosOcpSolver(mhe)
    return mhe_solver
    
