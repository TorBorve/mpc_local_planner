from cmath import pi
import statistics
import energyMPC
import numpy as np
import matplotlib.pyplot as plt
import time
from acados_template import AcadosOcp, AcadosSimSolver, AcadosModel, AcadosOcpSolver
import energyBicycleModel
import plotFnc
import yaml
from casadi import SX, vertcat, sin, cos, atan, tan


def main():
    ocp_solver = energyMPC.ocpSolver()
    ocp_integrator = AcadosSimSolver(ocp_solver.acados_ocp, 'acados_ocp_' + ocp_solver.acados_ocp.model.name + '.json')
    Nsim = 100 # Numer of iterations
    nx = ocp_solver.acados_ocp.model.x.size()[0]
    nu = ocp_solver.acados_ocp.model.u.size()[0]
    ny = nx + nu
    N = ocp_solver.acados_ocp.solver_options.qp_solver_cond_N
    Tf = ocp_solver.acados_ocp.solver_options.tf #Prediction horizon

    simX = np.ndarray((Nsim, nx))
    simU = np.ndarray((Nsim, nu))

    x0 = np.array([0, 0, 0, 0, 0, 0],float) # Initial state
    x_cur = x0
    simX[0,:] = x0

    computation = []
    time_solve = 0
    for i in range(Nsim):

        ocp_solver.set(0, "lbx", x_cur) 
        ocp_solver.set(0, "ubx", x_cur)
        
        start = time.time()
        solver_status = ocp_solver.solve() # Solve the ocp at current iteration
        t = time.time() - start
        computation.append(t)
        print(f'time: {t*1000:.2f}[ms], iter: {i}') #Calculate time it takes to solve iteration
        time_solve += t

        if solver_status != 0:
            print(f'solver error: {solver_status}')
            # raise Exception(f'solver error: {solver_status}')

        simU[i, :] = ocp_solver.get(0, "u") # Get latest input value
        
        ocp_integrator.set("x", x_cur) # Set the latest state as the current state
        ocp_integrator.set("u", simU[i, :]) # Set the latest input as current input

        integrator_status = ocp_integrator.solve() 

        if integrator_status != 0:
            raise Exception(f'integrator error: {integrator_status}')

        simX[i, :] = x_cur 
        x_cur = ocp_integrator.get("x") # Get latest state of system 


    print(f'avg. time: {time_solve/Nsim*1000}[ms]')
    print("Average speed:{}m/s".format(np.average(simX[:, 3])))
    t = np.linspace(0.0, Nsim * Tf / N, Nsim)
    print(len(t))
    #Plots

    plt.figure("Figure 1")
    plotFnc.trajcetory(t, simX)

    plt.figure("Figure 2")
    plotFnc.states(t, simX)

    plt.figure("Figure 3")
    plotFnc.computationalLoad(Nsim, computation)

    plt.figure("Figure 4")
    plotFnc.energyPlot(t, simX)

    plt.figure("Figure 5")
    plotFnc.inputs(t, simU)

    plt.show()


main()