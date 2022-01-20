from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, atan
import numpy as np

def carBicycleModel():
    modelName = "bicycle_model"

    Lf = 2.5
    #states
    x1 = SX.sym("x1") # x position
    y1 = SX.sym("y1") # y position
    psi = SX.sym("psi") # angle of car
    v = SX.sym("v") # velocity
    # cte = SX.sym("cte") # cross track error
    # epsi = SX.sym("epsi") # yaw error

    x = vertcat(x1, y1, psi, v)

    delta = SX.sym("delta") # steering angle
    a = SX.sym("a") # acceleration

    u = vertcat(delta, a)


    fExpl = vertcat(
            v*cos(psi),
            v*sin(psi),
            v/Lf*delta,
            a)
    
    model = AcadosModel()

    model.name = modelName
    model.f_expl_expr = fExpl
    model.x = x
    model.u = u
    model.p = []

    return model