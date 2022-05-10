from configparser import Interpolation
from re import S
from sqlite3 import paramstyle
from ssl import SSL_ERROR_WANT_X509_LOOKUP
from casadi import * #MX, vertcat, sin, cos, atan
from acados_template import AcadosOcp, AcadosSimSolver, AcadosModel, AcadosOcpSolver
import numpy as np
import scipy.linalg
import yaml
import os
from pathlib import Path

def getTrack(filename="LMS_Track.txt"):
    array=np.loadtxt(filename)
    sref=array[:,0]
    xref=array[:,1]
    yref=array[:,2]
    psiref=array[:,3]
    kapparef=array[:,4]
    return sref,xref,yref,psiref,kapparef


def bicycleModel(track="LMS_Track.txt"):

    with open("../params/mpc.yaml", "r") as paramFile:
            params = yaml.safe_load(paramFile)

    modelName = "parametric_path_bicycle_model"

# load track parameters
    [s0, _, _, _, kapparef] = getTrack(filename="LMS_Track.txt")
    length = len(s0)
    pathlength = s0[-1]
    # copy loop to beginning and end
    s0 = np.append(s0, [s0[length - 1] + s0[1:length]])
    kapparef = np.append(kapparef, kapparef[1:length])
    s0 = np.append([-s0[length - 2] + s0[length - 81 : length - 2]], s0) #Usikker på denne. Hvorfor veriden 81?
    kapparef = np.append(kapparef[length - 80 : length - 1], kapparef) # Hvorfor verdien 80?


    kapparef_s = interpolant("kapparef_s", "bspline", [s0], kapparef) # need to calculate kurvature
    #parameters
    C1 = 0.5 # this needs to be changed. Think it is lr/(lr + lf)
    C2 = 15.5 # not sure what this is
    m = 70 # mass of car
    
    #states
    s = MX.sym("s") # length of track
    n = MX.sym("n") # distance between cars position and centerline
    alpha = MX.sym("alpha") # heading angle of car - heading angle of centerline
    v = MX.sym("v") # velocity
    delta = MX.sym("delta") # steering angle
    throttle = MX.sym("throttle") # throttle

    x = vertcat(s, n, alpha, v, delta, throttle)

    #inputs
    deltaDotInput = MX.sym("delta_dot_input")
    throttleDotInput = MX.sym("throttle_dot_input")


    u = vertcat(deltaDotInput, throttleDotInput)

    sDot = MX.sym("s_dot")
    nDot = MX.sym("n_dot")
    alphaDot = MX.sym("alpha_dot")
    vDot = MX.sym("v_dot")
    deltaDotState = MX.sym("delta_dot_state")
    throttleDotState = MX.sym("throttle_dot_state")


    xDot = vertcat(sDot, nDot, alphaDot, vDot, deltaDotState, throttleDotState)


    sdota = (v * cos(alpha + C1 * delta)) / (1 - kapparef_s(s) * n)
    fExpl = vertcat(
            sdota,
            v * sin(alpha + C1 * delta),
            v * C2 * delta - kapparef_s(s) * sdota,
            5.0*throttle - 0.087*v,
            deltaDotInput,
            throttleDotInput)
    fImpl = xDot - fExpl            
    model = AcadosModel()

    p = vertcat(MX.sym("coeff_0"), MX.sym("coeff_1"), MX.sym("coeff_2"), MX.sym("coeff_3"))

    model.name = modelName
    model.f_expl_expr = fExpl
    model.f_impl_expr = fImpl
    model.xdot = xDot
    model.x = x
    model.u = u
    model.p = p
    model.con_h_expr = vertcat(n)

    return model

def costFunc(model):
    s = model.x[0]
    n = model.x[1]
    alpha = model.x[2]
    v = model.x[3]
    delta = model.x[4]
    throttle = model.x[5]
    deltaDot = model.u[0]
    throttleDot = model.u[1]
    coeffs = model.p
    
    
    # pathYaw = atan(3*coeffs[3]*x1*x1 + 2*coeffs[2]*x1 + coeffs[1]) 
    # epsi = psi - pathYaw
    # yPath = coeffs[3]*x1**3 + coeffs[2]*x1**2 + coeffs[1]*x1 + coeffs[0]
    # cte = yPath - y1
    return vertcat(n, alpha, v, delta, throttle, deltaDot, throttleDot)

def ocpSolver():
    with open("../params/mpc.yaml", "r") as paramFile:
            params = yaml.safe_load(paramFile)

    ocp = AcadosOcp()
    ocp.model = bicycleModel("LMS_Track.txt")
    
    N = params["mpc_N"]
    #N = 40
    dt = params["mpc_dt"]
    Tf = N*dt
    ocp.dims.N = N

    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]
    ny = nx + nu

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.yref = np.array([0, 0, 0, 0.5, 0, 0, 0, 0])
    #ocp.model.cost_y_expr = costFunc(ocp.model)
    ocp.cost.W = 2*np.diag([0, 50, 50, 10, 100, 1, 1, 1])

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[6, 0] = 1
    Vu[7, 1] = 1
    ocp.cost.Vu = Vu

    

    deltaMax = params["max_steering_angle"]
    deltaDotMax = params["max_steering_rotation_speed"]

    throttleMin = 0
    throttleMax = params["throttle_max"]
    throttleDotMax = params["throttle_dot_max"]

    widthTrack = params["width_track"]

    # constraint on n
    #ocp.model.con_h_expr = ocp.model
    #ocp.constraints.Jsbx = np.array([-widthTrack, widthTrack])
    #ocp.constraints.idxsbu = np.array([1])

    # constraints
    ocp.constraints.constr_type = "BGH"
    ocp.constraints.lbx = np.array([-deltaMax, throttleMin])
    ocp.constraints.ubx = np.array([deltaMax, throttleMax])
    ocp.constraints.idxbx = np.array([4, 5])
    ocp.constraints.lbu = np.array([-deltaDotMax, -throttleDotMax])
    ocp.constraints.ubu = np.array([deltaDotMax, throttleDotMax])
    ocp.constraints.idxbu = np.array([0, 1])

    # ocp.constraints.lsbx = np.array([-widthTrack])
    # ocp.constraints.usbx = np.array([widthTrack])
    # ocp.constraints.idxsbx = np.array([1])

    ocp.constraints.lh = np.array([-widthTrack])
    ocp.constraints.uh = np.array([widthTrack])

    

    ocp.cost.zl = 100 * np.ones((1,))
    ocp.cost.zu = 100 * np.ones((1,))
    ocp.cost.Zl = 0 * np.ones((1,))
    ocp.cost.Zu = 0 * np.ones((1,))

    ocp.constraints.idxsh = np.array([0])

    x0 = np.array([0, 0, 0, 0, 0, 0])
    ocp.constraints.x0 = x0

    param = np.array([0, -1, 0, 0.002])
    ocp.parameter_values = param

    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM" #"PARTIAL_CONDENSING_HPIPM" "FULL_CONDENSING_QPOASES" 
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.tf = Tf
    # ocp.solver_options.qp_solver_iter_max = 1000
    # ocp.solver_options.nlp_solver_max_iter = 2000

    ocp_solver = AcadosOcpSolver(ocp, 'acados_ocp_' + ocp.model.name + '.json')
    return ocp_solver #, params


bicycleModel("LMS_Track.txt")