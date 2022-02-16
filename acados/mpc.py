from configparser import Interpolation
from casadi import SX, vertcat, sin, cos, atan
from acados_template import AcadosOcp, AcadosSimSolver, AcadosModel, AcadosOcpSolver
import numpy as np
import scipy.linalg
import yaml

def bicycleModel(params):
    modelName = "bicycle_model"
    #distance between front and rear axle
    Lf = params["wheelbase"]
    #states
    x1 = SX.sym("x1") # x position
    y1 = SX.sym("y1") # y position
    psi = SX.sym("psi") # angle of car
    v = SX.sym("v") # velocity
    delta = SX.sym("delta") # steering angle
    throttle = SX.sym("throttle") # throttle

    x = vertcat(x1, y1, psi, v, delta, throttle)

    #inputs
    deltaDotInput = SX.sym("delta_dot_input")
    throttleDotInput = SX.sym("throttle_dot_input")


    u = vertcat(deltaDotInput, throttleDotInput)

    x1Dot = SX.sym("x1_dot")
    y1Dot = SX.sym("y1_dot")
    psiDot = SX.sym("psi_dot")
    vDot = SX.sym("v_dot")
    deltaDotState = SX.sym("delta_dot_state")
    throttleDotState = SX.sym("throttle_dot_state")



    xDot = vertcat(x1Dot, y1Dot, psiDot, vDot, deltaDotState, throttleDotState)


    fExpl = vertcat(
            v*cos(psi),
            v*sin(psi),
            v/Lf*delta,
            5.0*throttle - 0.087*v,
            deltaDotInput,
            throttleDotInput)
    fImpl = xDot - fExpl            
    model = AcadosModel()

    p = vertcat(SX.sym("coeff_0"), SX.sym("coeff_1"), SX.sym("coeff_2"), SX.sym("coeff_3"))

    model.name = modelName
    model.f_expl_expr = fExpl
    # model.f_impl_expr = fImpl
    model.xdot = xDot
    model.x = x
    model.u = u
    model.p = p

    return model

def costFunc(model):
    x1 = model.x[0]
    y1 = model.x[1]
    psi = model.x[2]
    v = model.x[3]
    delta = model.x[4]
    throttle = model.x[5]
    deltaDot = model.u[0]
    throttleDot = model.u[1]
    coeffs = model.p
    
    pathYaw = atan(3*coeffs[3]*x1*x1 + 2*coeffs[2]*x1 + coeffs[1])
    epsi = psi - pathYaw
    yPath = coeffs[3]*x1**3 + coeffs[2]*x1**2 + coeffs[1]*x1 + coeffs[0]
    cte = yPath - y1
    return vertcat(cte, epsi, v, delta, throttle, deltaDot, throttleDot)

def ocpSolver():
    with open("../params/mpc.yaml", "r") as paramFile:
            params = yaml.safe_load(paramFile)

    ocp = AcadosOcp()
    ocp.model = bicycleModel(params)
    
    N = params["mpc_N"]
    dt = params["mpc_dt"]
    Tf = N*dt
    ocp.dims.N = N

    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]
    ny = nx + nu

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.yref = np.array([0, 0, 6.0, 0, 0, 0, 0])
    ocp.model.cost_y_expr = costFunc(ocp.model)
    ocp.cost.W = 2*np.diag([500, 500, 100, 1, 10, 50, 1])

    deltaMax = params["max_steering_angle"]
    deltaDotMax = params["max_steering_rotation_speed"]
    throttleMin = 0.0
    throttleMax = params["throttle_max"]
    throttleDotMax = params["throttle_dot_max"]   
    ocp.constraints.constr_type = "BGH"
    ocp.constraints.lbx = np.array([-deltaMax, throttleMin])
    ocp.constraints.ubx = np.array([deltaMax, throttleMax])
    ocp.constraints.idxbx = np.array([4, 5])
    ocp.constraints.lbu = np.array([-deltaDotMax, -throttleDotMax])
    ocp.constraints.ubu = np.array([deltaDotMax, throttleDotMax])
    ocp.constraints.idxbu = np.array([0, 1])

    x0 = np.array([-10, 0, 0, 0, 0, 0])
    ocp.constraints.x0 = x0

    param = np.array([0, -1, 0, 0.002])
    ocp.parameter_values = param

    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM" #"PARTIAL_CONDENSING_HPIPM" "FULL_CONDENSING_QPOASES" 
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.tf = Tf
    # ocp.solver_options.qp_solver_iter_max = 1000
    # ocp.solver_options.nlp_solver_max_iter = 2000

    ocp_solver = AcadosOcpSolver(ocp, 'acados_ocp_' + ocp.model.name + '.json')
    return ocp_solver