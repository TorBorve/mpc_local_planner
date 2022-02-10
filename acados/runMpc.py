import mpc
from acados_template import AcadosOcp, AcadosSimSolver, AcadosModel, AcadosOcpSolver
import numpy as np
import matplotlib.pyplot as plt
import time

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
    ocp_solver = mpc.ocpSolver()
    ocp_integrator = AcadosSimSolver(ocp_solver.acados_ocp, 'acados_ocp_' + ocp_solver.acados_ocp.model.name + '.json')
    Nsim = 200
    nx = ocp_solver.acados_ocp.model.x.size()[0]
    nu = ocp_solver.acados_ocp.model.u.size()[0]
    ny = nx + nu
    N = ocp_solver.acados_ocp.solver_options.qp_solver_cond_N
    Tf = ocp_solver.acados_ocp.solver_options.tf

    simX = np.ndarray((Nsim + 1, nx))
    simU = np.ndarray((Nsim, nu))

    x0 = np.array([-30, 0, 0, 0, 0, 0])
    x_cur = x0
    simX[0,:] = x0

    time_solve = 0
    for i in range(Nsim):

        ocp_solver.set(0, "lbx", x_cur)
        ocp_solver.set(0, "ubx", x_cur)

        # if (i >= Nsim / 2):
        #     for j in range(N):
        #         ocp_solver.set(j, "p", np.zeros(4))

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
        simX[i + 1, :] = x_cur
    
    print(f'avg. time: {time_solve/Nsim*1000}[ms]')
    plot(np.linspace(0, Tf/N*Nsim, Nsim + 1), simX)
    return

main()