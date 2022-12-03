from casadi import SX, vertcat, sin, cos, atan, tan
from acados_template import AcadosOcp, AcadosSimSolver, AcadosModel, AcadosOcpSolver
import numpy as np
import scipy.linalg
import yaml
def bicycleModel(params):
    modelName = "path_tracking"
    
    Lf = params["wheelbase"] #distance between front and rear axle
    r = params["radius"] # Wheel radius
    l = params["wheelbase"] # Distance between front wheel and rear wheel
    G = params["gear_ratio"] # Gear ratio
    m = params["mass_car"]# Mass of car
    V = params["voltage"] # Voltage
    Cd = params["drag_coefficient"] # Drag coefficient
    rho = 1.2 # Air density [kg/m³]
    A = params["frontal_area"]# Frontal area [m²]

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

    fExpl = vertcat(
            v*cos(psi),
            v*sin(psi),
            v/Lf*tan(delta),
            5.0*throttle - 0.087*v + sin(pitch)*9.81, #V * 3.2 * throttle * r / (v * G * m + 1) - (1/2*(rho*Cd*A*(v)**2) / m) + sin(pitch)*9.81,
            deltaDotInput,
            throttleDotInput)
    fImpl = xDot - fExpl            
    model = AcadosModel()

    p = vertcat(SX.sym("coeff_0"), SX.sym("coeff_1"), SX.sym("coeff_2"), SX.sym("coeff_3"), pitch, SX.sym("v_ref"))

    model.name = modelName
    model.f_expl_expr = fExpl
    model.f_impl_expr = fImpl
    model.xdot = xDot
    model.x = x
    model.u = u
    model.p = p
    # model.con_h_expr = vertcat(v)

    return model

def costFunc(model, params):

    r = params["radius"] # Radius of wheel [m]
    m = params["mass_car"] # Mass of vehicle [kg]
    Cr = params["rolling_resistance"] # Rolling resistance
    Cd = params["drag_coefficient"] # Drag coefficient
    rho = 1.2 # Air density [kg/m³]
    A = params["frontal_area"] # Frontal area [m²]
    g = 9.81

    x1 = model.x[0]
    y1 = model.x[1]
    psi = model.x[2]
    v = model.x[3]
    delta = model.x[4]
    throttle = model.x[5]
    deltaDot = model.u[0]
    throttleDot = model.u[1]
    a = model.f_expl_expr[3]
    coeffs = model.p

    Tm = r * (m*a + m*g*Cr + 1/2*rho*Cd*A*(v)**2)
    energy = Tm * v/r
    
    pathYaw = atan(3*coeffs[3]*x1*x1 + 2*coeffs[2]*x1 + coeffs[1])
    epsi = psi - pathYaw
    yPath = coeffs[3]*x1**3 + coeffs[2]*x1**2 + coeffs[1]*x1 + coeffs[0]
    cte = yPath - y1
    return vertcat(cte, epsi, v - model.p[5], delta, throttle, deltaDot, throttleDot)

def ocpSolver():
    with open("../../build/auto_gen.yaml", "r") as paramFile:
            params = yaml.safe_load(paramFile)
    params = params["/mpc_local_planner"]["mpc_local_planner"]["ros__parameters"]


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
    ocp.cost.yref = np.array([0, 0, 0, 0, 0, 0, 0])
    ocp.model.cost_y_expr = costFunc(ocp.model, params)
    # ocp.cost.W = np.diag([5, 35, 10, 0, 0, 0, 0, 0.00001]) # Energy Mode
    ocp.cost.W = np.diag([5, 5, 10, 0.01, 0.1, 0.5, 0.1]) # Not Energy Mode

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
    ocp.constraints.ubu = np.array([deltaDotMax, throttleDotMax])
    ocp.constraints.idxbu = np.array([0, 1])

    # ocp.cost.zl = 1000 * np.ones((1,))
    # ocp.cost.Zl = 0 * np.ones((1,))
    # ocp.cost.zu = 1000 * np.ones((1,))
    # ocp.cost.Zu = 0 * np.ones((1,))
    # ocp.constraints.lh = np.array([0.0])
    # ocp.constraints.uh = np.array([6.0])

    # ocp.constraints.idxsh = np.array([0])


    x0 = np.array([-10, 0, 0, 0, 0, 0])
    ocp.constraints.x0 = x0

    param = np.array([0, -1, 0, 0.002, 0.0, 4.0])
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