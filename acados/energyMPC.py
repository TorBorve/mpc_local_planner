from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from attr import NOTHING
import numpy as np
import energyBicycleModel 
import scipy.linalg


def ocpSolver():
    # Create render arguments
    ocp = AcadosOcp()
    
    # export model
    ocp.model = energyBicycleModel.energyBicycleModel2()

    # Set dimensions
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]
    ny = nx + nu

    N = 40 # Number of steps
    dt = 0.1 # Time steps
    Tf = N*dt

    ocp.dims.N = N

    # Define acados ODE
    # model_ocp = AcadosModel()
    # model_ocp.f_impl_expr = model.f_impl_expr
    # model_ocp.f_expl_expr = model.f_expl_expr
    # model_ocp.x = model.x
    # model_ocp.u = model.u
    # model_ocp.name = model.name
    # ocp.model = model_ocp

    # Define constraint

    deltaMax = 0.57
    deltaDotMax = 1

    throttleMin = 0
    throttleMax = 0.5
    throttleDotMax = 0.33

    ocp.constraints.constr_type = "BGH"
    ocp.constraints.lbx = np.array([-deltaMax, throttleMin])
    ocp.constraints.ubx = np.array([deltaMax, throttleMax])
    ocp.constraints.idxbx = np.array([4, 5])
    ocp.constraints.lbu = np.array([-deltaDotMax, -throttleDotMax])
    ocp.constraints.ubu = np.array([deltaDotMax, throttleDotMax])
    ocp.constraints.idxbu = np.array([0, 1])

    x0 = np.array([0, 0, 0, 0, 0, 0])
    ocp.constraints.x0 = x0

    # Vu = np.zeros((ny, nu))
    # Vu[6, 0] = 1
    # Vu[7, 1] = 1
    # ocp.cost.Vu = Vu




    # Cost

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.yref = np.array([0, 0, 6, 0, 0, 0, 0, 0])
    ocp.model.cost_y_expr = energyBicycleModel.costFnc2(ocp.model)
    ocp.cost.W = 2*np.diag([500, 500, 100, 1, 10, 50, 1, 0])


    # Set QP solver and integration
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM" #"PARTIAL_CONDENSING_HPIPM" #"FULL_CONDENSING_QPOASES"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    # ocp.solver_options.sim_method_num_stages = 4
    # ocp.solver_options.sim_method_num_steps = 3

    ocp_solver = AcadosOcpSolver(ocp, 'acados_ocp_' + ocp.model.name + '.json')

    return ocp_solver

ocpSolver()