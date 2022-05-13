from cmath import pi
import statistics
import energyMPC
import numpy as np
import matplotlib.pyplot as plt
import time
from acados_template import AcadosOcp, AcadosSimSolver, AcadosModel, AcadosOcpSolver
import energyBicycleModel

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

def main():
    ocp_solver = energyMPC.ocpSolver()
    ocp_integrator = AcadosSimSolver(ocp_solver.acados_ocp, 'acados_ocp_' + ocp_solver.acados_ocp.model.name + '.json')
    Nsim = 200
    nx = ocp_solver.acados_ocp.model.x.size()[0]
    nu = ocp_solver.acados_ocp.model.u.size()[0]
    ny = nx + nu
    N = ocp_solver.acados_ocp.solver_options.qp_solver_cond_N
    Tf = ocp_solver.acados_ocp.solver_options.tf

    simX = np.ndarray((Nsim, nx))
    simU = np.ndarray((Nsim, nu))

    x0 = np.array([0, 5, 0, 0, 0, 0])
    x_cur = x0
    simX[0,:] = x0
    
    time_solve = 0
    for i in range(Nsim):

        ocp_solver.set(0, "lbx", x_cur)
        ocp_solver.set(0, "ubx", x_cur)

        # if (i >= Nsim / 2):
        #     for j in range(N):
        #         ocp_solver.set(j, "p", np.zeros(4))

        #AcadosOcpSolver.get_stats("statistics")
        
        start = time.time()
        solver_status = ocp_solver.solve()
        t = time.time() - start
        print(f'time: {t*1000:.2f}[ms], iter: {i}')
        time_solve += t

        if solver_status != 0:
            print(f'solver error: {solver_status}')
            # raise Exception(f'solver error: {solver_status}')

        simU[i, :] = ocp_solver.get(0, "u")
        
        ocp_integrator.set("x", x_cur)
        ocp_integrator.set("u", simU[i, :])

        integrator_status = ocp_integrator.solve()

        if integrator_status != 0:
            raise Exception(f'integrator error: {integrator_status}')
        
        x_cur = ocp_integrator.get("x")
        simX[i, :] = x_cur
        

    
    print(f'avg. time: {time_solve/Nsim*1000}[ms]')
    print("Average speed:{}m/s".format(np.average(simX[:, 3])))
    t = np.linspace(0.0, Nsim * Tf / N, Nsim + 1)
    plot(t, simX)
    plt.show()


main()