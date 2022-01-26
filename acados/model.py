from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, atan
import numpy as np

def carBicycleModel():
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