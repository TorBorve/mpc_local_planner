from re import L, M
from socket import NI_NUMERICHOST
from this import d
from xml.etree.ElementTree import C14NWriterTarget
from attr import NOTHING
import numpy as np
from casadi import *
from acados_template import AcadosOcp, AcadosSimSolver, AcadosModel, AcadosOcpSolver
from requests import delete
import yaml 


def energyBicycleModel(params):
    
    modelName = "BicycleModel"

    # Constants

    

    r = params["radius"] # Wheel radius
    l = params["wheelbase"] # Distance between front wheel and rear wheel
    G = params["gear_ratio"] # Gear ratio
    m = params["mass_car"]# Mass of car
    V = params["voltage"] # Voltage
    Cd = params["drag_coefficient"] # Drag coefficient
    rho = params["air_resistance"]# Air resistance [kg/m³]
    A = params["frontal_area"]# Frontal area [m²]

    # States

    x1 = SX.sym("x1") # X-position
    y1 = SX.sym("y1") # Y-position
    psi = SX.sym("psi") # Angle of car
    v = SX.sym("v") # Velocity of car
    #omega = SX.sym("omega") # Angular velocity
    delta = SX.sym("delta") # Steering angle
    throttle = SX.sym("throttle")


    x = vertcat(x1, y1, psi, v, delta, throttle)

    # Input
    
    #omega_dot = SX.sym("omega_dot") # Angular acceleration
    delta_dot = SX.sym("delta_dot") # Steering angle derivative
    throttle_dot = SX.sym("throttle_dot")

    u = vertcat(delta_dot, throttle_dot)

    x1Dot = SX.sym("x1_dot")
    y1Dot = SX.sym("y1_dot")
    psiDot = SX.sym("psi_dot")
    a = SX.sym("a")
    delta_dot_state = SX.sym("delta_dot_state")
    throttle_dot_state = SX.sym("throttle_dot_state")

    xDot = vertcat(x1Dot, y1Dot, psiDot, a, delta_dot_state, throttle_dot_state)

    f_expl = vertcat(
        v * (cos(psi)), # * cos(psi) - sin(delta)/2 * sin(psi)),
        v * (sin(psi)), # * sin(psi) + sin(delta)/2 * cos(psi)),
        v * sin(delta)/l,
        V * 3.2 * throttle * r / (v * G * m + 1) - (1/2*(rho*Cd*A*(v)**2) / m), #5*throttle - 0.03*v,
        delta_dot,
        throttle_dot
    )    

    p = vertcat(SX.sym("coeff_0"), SX.sym("coeff_1"), SX.sym("coeff_2"), SX.sym("coeff_3"))

    f_impl = xDot - f_expl
    model = AcadosModel()

    model.name = modelName
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    model.xdot = xDot
    model.x = x
    model.u = u
    model.p = p

    return model

def costFnc(model):

    with open("../params/mpc.yaml", "r") as paramFile:
        params = yaml.safe_load(paramFile)

    r = params["radius"] # Radius of wheel [m]
    m = params["mass_car"] # Mass of vehicle [kg]
    Cr = params["rolling_resistance"] # Rolling resistance
    Cd = params["drag_coefficient"] # Drag coefficient
    rho = params["air_resistance"] # Air resistance [kg/m³]
    A = params["frontal_area"] # Frontal area [m²]
    g = params["gravity_constant"]# Gravity constant [m/s²]
    G = params["total_reduction_ratio"]# Total reduction ratio


    x1 = model.x[0]
    y1 = model.x[1]
    psi = model.x[2]
    v = model.x[3]
    delta = model.x[4]
    throttle = model.x[5]

    delta_dot = model.u[0]
    throttle_dot = model.u[1]

    a = model.f_expl_expr[3]

    coeffs = model.p


    Tm = r/G * (m*a + m*g*Cr + 1/2*rho*Cd*A*(v)**2)

    energy = Tm * v/r

    pathYaw = atan(3*coeffs[3]*x1*x1 + 2*coeffs[2]*x1 + coeffs[1]) 
    #pathYaw = 0 
    epsi = psi - pathYaw
    yPath =  coeffs[3]*x1**3 + coeffs[2]*x1**2 + coeffs[1]*x1 + coeffs[0]
    #yPath = 0
    cte = yPath - y1

    return vertcat(cte, epsi, v, delta, throttle, delta_dot, throttle_dot, energy)
