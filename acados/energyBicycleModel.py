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
    
    modelName = "BicycleModel" # Name of model

    # Constants

    r = params["radius"] # Wheel radius [m]
    l = params["wheelbase"] # Distance between front wheel and rear wheel [m]
    G = params["gear_ratio"] # Gear ratio
    m = params["mass_car"]# Mass of car [kg]
    V = params["voltage"] # Voltage
    Cd = params["drag_coefficient"] # Drag coefficient
    rho = params["air_resistance"]# Air resistance [kg/m³]
    A = params["frontal_area"]# Frontal area [m²]

    # States

    x1 = SX.sym("x1") # X-position
    y1 = SX.sym("y1") # Y-position
    psi = SX.sym("psi") # Angle of car
    v = SX.sym("v") # Velocity of car
    delta = SX.sym("delta") # Steering angle
    throttle = SX.sym("throttle") # Throttle


    x = vertcat(x1, y1, psi, v, delta, throttle) 

    # Input
    
    delta_dot = SX.sym("delta_dot") # Steering angle derivative
    throttle_dot = SX.sym("throttle_dot") # Throttle derivative

    u = vertcat(delta_dot, throttle_dot)

    x1Dot = SX.sym("x1_dot")
    y1Dot = SX.sym("y1_dot")
    psiDot = SX.sym("psi_dot")
    a = SX.sym("a")
    delta_dot_state = SX.sym("delta_dot_state")
    throttle_dot_state = SX.sym("throttle_dot_state")

    xDot = vertcat(x1Dot, y1Dot, psiDot, a, delta_dot_state, throttle_dot_state)

    f_expl = vertcat(
        v * (cos(psi)), # x_dot
        v * (sin(psi)), # y_dot
        v * tan(delta)/l, # psi_dot
        V * 3.2 * throttle * r / (v * G * m + 1) - (1/2*(rho*Cd*A*(v)**2) / m), # a
        delta_dot,
        throttle_dot
    )    
    #Parameters used in describing third-order polynomial
    p = vertcat(SX.sym("coeff_0"), SX.sym("coeff_1"), SX.sym("coeff_2"), 
                SX.sym("coeff_3"))

    f_impl = xDot - f_expl
    model = AcadosModel() # Class containing all information of the model

    # Add the respective states, inputs and so on to the different model
    model.name = modelName
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    model.xdot = xDot
    model.x = x
    model.u = u
    model.p = p

    return model

def costFnc(model):

    # Opem yaml file with different constants
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

    # Define what is going to be in cost function
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


    Tm = r/G * (m*a + m*g*Cr + 1/2*rho*Cd*A*(v)**2) # Torque

    energy = Tm * v/r # Energy equation

    # Different paths to test

    # Sinusfunction
    yPath = 10*sin(x1/10) 
    pathYaw = atan(cos(x1/10)) 

    # Third-order polynomial
    # yPath =  coeffs[3]*x1**3 + coeffs[2]*x1**2 + coeffs[1]*x1 + coeffs[0] 
    # pathYaw = atan(3*coeffs[3]*x1*x1 + 2*coeffs[2]*x1 + coeffs[1])  

    # Straight line
    # yPath = 0 
    # pathYaw = 0 

    # Cross-track error and heading error
    epsi = psi - pathYaw
    cte = yPath - y1

    # What variables we want to minimize
    return vertcat(cte, epsi, v, delta, throttle, delta_dot, throttle_dot, energy)
