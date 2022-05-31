from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from attr import NOTHING
import numpy as np
import energyBicycleModel 
import scipy.linalg
import yaml


def ocpSolver():
    # Open params file
    with open("../params/mpc.yaml", "r") as paramFile:
        params = yaml.safe_load(paramFile)

    # Create render arguments
    ocp = AcadosOcp()
    
    # export model
    ocp.model = energyBicycleModel.energyBicycleModel(params)

    # Set dimensions
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]
    ny = nx + nu

    N = params["mpc_N"] # Number of steps
    dt = params["mpc_dt"] # Time steps
    Tf = N*dt

    ocp.dims.N = N

    # Constraints
    deltaMax = params["max_steering_angle"]
    deltaDotMax = params["max_steering_rotation_speed"]

    throttleMin = 0
    throttleMax = params["throttle_max"]
    throttleDotMax = params["throttle_dot_max"]

    ocp.constraints.constr_type = "BGH"
    ocp.constraints.lbx = np.array([-deltaMax, throttleMin]) # Lower bound on state
    ocp.constraints.ubx = np.array([deltaMax, throttleMax]) # Upper bound on state
    ocp.constraints.idxbx = np.array([4, 5])
    ocp.constraints.lbu = np.array([-deltaDotMax, -throttleDotMax]) # Lower bound on input
    ocp.constraints.ubu = np.array([deltaDotMax, throttleDotMax]) # Upper bound on input
    ocp.constraints.idxbu = np.array([0, 1])

    x0 = np.array([0, 0, 0, 0, 0, 0])
    ocp.constraints.x0 = x0

    param = np.array([0, 0, 0, 0])
    ocp.parameter_values = param

    # Cost

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.yref = np.array([0, 0, 10, 0, 0, 0, 0, 0]) # Reference the different variables in cost function should follow
    ocp.model.cost_y_expr = energyBicycleModel.costFnc(ocp.model)
    ocp.cost.W = np.diag([5, 35, 10, 0, 0, 1, 1, 0.01]) # Weigths when in Energy mode
    #ocp.cost.W = np.diag([5, 35, 10, 0, 0, 1, 1, 0]) # Weights when not in Energy mode

    # Set QP solver and integration
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.qp_solver =  "FULL_CONDENSING_HPIPM" #"PARTIAL_CONDENSING_HPIPM" #"FULL_CONDENSING_QPOASES"
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    # ocp.solver_options.sim_method_num_stages = 4
    # ocp.solver_options.sim_method_num_steps = 3

    # Create json file
    ocp_solver = AcadosOcpSolver(ocp, 'acados_ocp_' + ocp.model.name + '.json')

    return ocp_solver

ocpSolver()