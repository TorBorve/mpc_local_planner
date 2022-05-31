import matplotlib.pyplot as plt
import numpy as np
import yaml
from casadi import SX, vertcat, sin, cos, atan, tan

def plot(shootingNodes, simX):
    x1 = simX[:, 0]
    y1 = simX[:, 1]
    # psi = simX[:, ]

    plt.xlim([min(x1), max(x1)])
    plt.ylim([min(y1), max(y1)])

    for i in range(simX.shape[0]):
        plt.plot(x1[:i], y1[:i])
        plt.pause(shootingNodes[1])
    plt.show()

def trajcetory(t, simX):

    reference_path_straight_line = [0] * len(t)
    reference_path_sinus = 10*sin(simX[:,0]/10)
    x1 = simX[:, 0]
    y1 = simX[:, 1]


    plt.plot(x1, reference_path_sinus)
    plt.plot(x1, y1)
    plt.title("Path Following")
    plt.legend(["Reference path", "Actual path"])
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")

def states(t, simX):


    # States
    psi = simX[:,2]
    v = simX[:, 3]
    delta = simX[:, 4]
    throttle = simX[:, 5]

    # Plot 1
    plt.subplot(2, 2, 1)
    plt.plot(t, psi)
    plt.title("Heading angle")
    plt.legend(["psi"])
    plt.xlabel("Time [s] ")
    plt.ylabel("Angle [rad]")

    # Plot 2
    plt.subplot(2, 2, 2)
    plt.plot(t, delta)
    plt.title("Angle of wheels")
    plt.legend(["delta"])
    plt.xlabel("Time [s] ")
    plt.ylabel("Angle [rad]")

    # Plot 3
    plt.subplot(2, 2, 3)
    plt.plot(t, throttle)
    plt.title("Throttle")
    plt.xlabel("Time [s]")
    plt.ylabel("Input")

    # Plot 4
    plt.subplot(2, 2, 4)
    plt.plot(t, v)
    plt.title("Velocity")
    plt.xlabel("Time [s] ")
    plt.ylabel("Velocity [m/s]")

    plt.suptitle("States")

def inputs(t, simU):

    # Inputs
    delta_dot = simU[:,0]
    throttle_dot = simU[:,1]

    # Bounds
    delta_dot_bound_upper = [0.8] * len(t)
    delta_dot_bound_lower = [-0.8] * len(t)
    
    throttle_dot_bound_upper = [0.33] * len(t)
    throttle_dot_bound_lower = [-0.33] * len(t)

    # Plot 1
    plt.subplot(1,2,1)
    plt.plot(t, delta_dot)
    plt.plot(t, delta_dot_bound_upper)
    plt.plot(t, delta_dot_bound_lower)
    
    plt.title("Steering angle rate")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle rate [rad/s]")
    plt.legend(["delta_dot", "upper bound", "lower bound"], loc="upper right")

    # Plot 2
    plt.subplot(1,2,2)
    plt.plot(t, throttle_dot)
    plt.plot(t, throttle_dot_bound_upper)
    plt.plot(t, throttle_dot_bound_lower)

    plt.title("Throttle rate")
    plt.xlabel("Time [s]")
    plt.ylabel("Input rate [1/s]")
    plt.legend(["throttle_dot", "upper bound", "lower bound"],loc="upper right")

    plt.suptitle("Inputs")

def computationalLoad(iteration, computation):
    iterations = []
    for i in range(iteration):
        iterations.append(i)
    plt.plot(iterations, computation)
    plt.title("Computational Load")
    plt.legend(["Time"])
    plt.xlabel("Iteration")
    plt.ylabel("Time [s]")

def energyPlot(t, simX):

    with open("../params/mpc.yaml", "r") as paramFile:
        params = yaml.safe_load(paramFile)

    dt = 0.15
    r = params["radius"] # Wheel radius
    l = params["wheelbase"] # Distance between front wheel and rear wheel
    G = params["gear_ratio"] # Gear ratio
    m = params["mass_car"]# Mass of car
    V = params["voltage"] # Voltage
    Cd = params["drag_coefficient"] # Drag coefficient
    rho = params["air_resistance"]# Air resistance [kg/m³]
    A = params["frontal_area"]# Frontal area [m²]
    g = params["gravity_constant"]# Gravity constant [m/s²]
    Cr = params["rolling_resistance"] # Rolling resistance

    x1 = simX[:,0]
    throttle = simX[:, 5]
    v = simX[:, 3]

    energy = [0]
    energy_temp = 0
    i_temp = 1
    
    for i in range(len(simX)):
        if x1[i] < 50:
            a = V * 3.2 * throttle[i] * r / ( v[i]* G * m + 1) - (1/2*(rho*Cd*A*(v[i])**2) / m) # Acceleration

            Tm = r/G * (m*a + m*g*Cr + 1/2*rho*Cd*A*(v[i])**2) # Torque

            energy_temp += Tm * v[i]/r * dt

            energy.append(energy_temp)

            i_temp += 1

        else:
            break
    distance = x1[0:i_temp]
    print(len(distance), len(energy))       
    plt.plot(distance, energy)
    plt.legend(["Energy"])
    plt.xlabel("Longitudinal distance [m]")
    plt.ylabel("Energy [Joule]")
    plt.title("Energy Used")
    



