import os
from acados_template import AcadosSim,AcadosSimSolver


def setSim(model_sim,param):
    acados_source_path = os.environ['ACADOS_SOURCE_DIR']
    acados_include_path = acados_source_path + '/include'
    acados_lib_path = acados_source_path + '/lib'
    sim = AcadosSim()
    sim.acados_include_path = acados_include_path
    sim.acados_lib_path = acados_lib_path
    sim.model = model_sim.model
    sim.solver_options.T = param.Tsim
    sim.solver_options.integrator_type = model_sim.integrator_type
    sim_solver = AcadosSimSolver(sim)
    
    return sim_solver