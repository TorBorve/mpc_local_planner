from casadi import SX, vertcat, sin, cos, atan
from acados_template import AcadosOcp, AcadosSimSolver, AcadosModel, AcadosOcpSolver
import numpy as np
import scipy.linalg

def bicycleModel():
    modelName = "bicycle_model"

    Lf = 2.65
    #states
    x1 = SX.sym("x1") # x position
    y1 = SX.sym("y1") # y position
    psi = SX.sym("psi") # angle of car
    v = SX.sym("v") # velocity
    # cte = SX.sym("cte") # cross track error
    # epsi = SX.sym("epsi") # yaw error
    delta = SX.sym("delta") # steering angle
    a = SX.sym("a") # acceleration

    x = vertcat(x1, y1, psi, v, delta)

    deltaDotInput = SX.sym("delta_dot")


    u = vertcat(deltaDotInput, a)

    x1Dot = SX.sym("x1_dot")
    y1Dot = SX.sym("y1_dot")
    psiDot = SX.sym("psi_dot")
    vDot = SX.sym("v_dot")
    deltaDotState = SX.sym("delta_dot_state")


    xDot = vertcat(x1Dot, y1Dot, psiDot, vDot, deltaDotState)

    fExpl = vertcat(
            v*cos(psi),
            v*sin(psi),
            v/Lf*delta,
            a,
            deltaDotInput)
    fImpl = xDot - fExpl            
    model = AcadosModel()

    p = vertcat(SX.sym("coeff_0"), SX.sym("coeff_1"), SX.sym("coeff_2"), SX.sym("coeff_3"))

    model.name = modelName
    model.f_expl_expr = fExpl
    model.f_impl_expr = fImpl
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
    deltaDot = model.u[0]
    a = model.u[1]
    coeffs = model.p
    
    pathYaw = atan(3*coeffs[3]*x1*x1 + 2*coeffs[2]*x1 + coeffs[1])
    epsi = psi - pathYaw
    yPath = coeffs[3]*x1**3 + coeffs[2]*x1**2 + coeffs[1]*x1 + coeffs[0]
    cte = yPath - y1
    return vertcat(cte, epsi, v, delta, deltaDot, a)

def ocpSolver():
    ocp = AcadosOcp()
    ocp.model = bicycleModel()

    Tf = 2.0
    N = 20
    ocp.dims.N = N

    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]
    ny = nx + nu

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.yref = np.array([0, 0, 4, 0, 0, 0])
    ocp.model.cost_y_expr = costFunc(ocp.model)
    ocp.cost.W = 2*np.diag([10, 100, 1, 0.1, 0.1, 0.1])

    aMax = 1
    deltaMax = 0.57
    deltaDotMax = 0.8
    ocp.constraints.constr_type = "BGH"
    ocp.constraints.lbx = np.array([-deltaMax])
    ocp.constraints.ubx = np.array([deltaMax])
    ocp.constraints.idxbx = np.array([4])
    ocp.constraints.lbu = np.array([-deltaDotMax, -aMax])
    ocp.constraints.ubu = np.array([deltaDotMax, aMax])
    ocp.constraints.idxbu = np.array([0, 1])

    x0 = np.array([-10, 0, 0, 0, 0])
    ocp.constraints.x0 = x0

    param = np.array([0, -1, 0, 0.002])
    ocp.parameter_values = param

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM" #"FULL_CONDENSING_QPOASES" "FULL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.tf = Tf
    # ocp.solver_options.qp_solver_iter_max = 1000
    # ocp.solver_options.nlp_solver_max_iter = 2000

    ocp_solver = AcadosOcpSolver(ocp, 'acados_ocp_' + ocp.model.name + '.json')
    return ocp_solver