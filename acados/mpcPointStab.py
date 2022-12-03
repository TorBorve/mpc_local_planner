from casadi import SX, vertcat, sin, cos, atan
from acados_template import AcadosOcp, AcadosSimSolver, AcadosModel, AcadosOcpSolver
import numpy as np
import scipy.linalg
import yaml

def bicycleModel(params):
    modelName = "point_stab"
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

    pitch = SX.sym("pitch")

    a = 5.0*throttle - 0.087*v + sin(pitch)*9.81

    fExpl = vertcat(
            v*cos(psi),
            v*sin(psi),
            v/Lf*delta,
            a,
            deltaDotInput,
            throttleDotInput)
    fImpl = xDot - fExpl            
    model = AcadosModel()

    p = vertcat(SX.sym("x_ref"), SX.sym("y_ref"), SX.sym("psi_ref"), pitch)

    model.name = modelName
    model.f_expl_expr = fExpl
    model.f_impl_expr = fImpl
    model.xdot = xDot
    model.x = x
    model.u = u
    model.p = p
    model.con_h_expr = vertcat(a)

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
    xRef = model.p[0]
    yRef = model.p[1]
    psiRef = model.p[2]
    
    return vertcat(x1 - xRef, y1 - yRef, psi - psiRef, v, delta, throttle, deltaDot, throttleDot)

def ocpSolver():
    with open("../../build/auto_gen.yaml", "r") as paramFile:
            params = yaml.safe_load(paramFile)
    params = params["/mpc_local_planner"]["mpc_local_planner"]["ros__parameters"]
    ocp = AcadosOcp()
    ocp.model = bicycleModel(params)
    p = ocp.model.p
    
    N = params["mpc_N"]
    dt = params["mpc_dt"]
    Tf = N*dt
    ocp.dims.N = N

    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]
    ny = nx + nu

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.model.cost_y_expr = costFunc(ocp.model)
    ocp.cost.yref = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    Q = 2*np.diag([5, 5, 10, 10, 0.01, 1])
    R = 2*np.diag([0.1, 0.1])
    ocp.cost.W = scipy.linalg.block_diag(Q, R)
#     ocp.cost.Vx = np.zeros((ny, nx))
#     ocp.cost.Vx[:nx, :nx] = np.eye(nx)
#     ocp.cost.Vu = np.zeros((ny, nu))
#     ocp.cost.Vu[6, 0] = 1
#     ocp.cost.Vu[7, 1] = 1


    deltaMax = params["max_steering_angle"]
    deltaDotMax = params["max_steering_rotation_speed"]
    throttleMin = params["throttle_min"]
    throttleMax = params["throttle_max"]
    throttleDotMax = params["throttle_dot_max"]   
    ocp.constraints.constr_type = "BGH"
    ocp.constraints.lbx = np.array([-deltaMax, throttleMin])
    ocp.constraints.ubx = np.array([deltaMax, throttleMax])
    ocp.constraints.idxbx = np.array([4, 5])
    ocp.constraints.lbu = np.array([-deltaDotMax, -throttleDotMax])
    ocp.constraints.ubu = np.array([deltaDotMax/4, throttleDotMax])
    ocp.constraints.idxbu = np.array([0, 1])

    # Soft constraints
    ocp.cost.zl = 100 * np.ones((1,))
    ocp.cost.Zl = 0 * np.ones((1,))
    ocp.cost.zu = 100 * np.ones((1,))
    ocp.cost.Zu = 0 * np.ones((1,))
    ocp.constraints.lh = np.array([-100])
    ocp.constraints.uh = np.array([0.1])

    ocp.constraints.idxsh = np.array([0])

    x0 = np.array([-10, 0, 0, 0, 0, 0])
    ocp.constraints.x0 = x0

    param = np.array([0, 0, 0, 0])
    ocp.parameter_values = param

    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM" #"PARTIAL_CONDENSING_HPIPM" "FULL_CONDENSING_QPOASES" 
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = params["nlp_solver_type"]
    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.tf = Tf

    ocp_solver = AcadosOcpSolver(ocp, 'acados_ocp_' + ocp.model.name + '.json')
    return ocp_solver

ocpSolver()