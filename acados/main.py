from model import carBicycleModel
from acados_template import AcadosOcp, AcadosSimSolver, AcadosModel, AcadosOcpSolver
import numpy as np
import scipy.linalg
from casadi import SX, vertcat, sin, cos, Function, atan
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


def costFunc(model):
    x1 = model.x[0]
    y1 = model.x[1]
    psi = model.x[2]
    v = model.x[3]
    delta = model.u[0]
    a = model.u[1]
    
    vRef = 4
    coeffs = np.array([0, -1, 0, 0.002])
    pathYaw = atan(3*coeffs[3]*x1*x1 + 2*coeffs[2]*x1 + coeffs[1])
    epsi = psi - pathYaw
    yPath = coeffs[3]*x1**3 + coeffs[2]*x1**2 + coeffs[1]*x1 + coeffs[0]
    cte = yPath - y1
    # return 10*y1**2 + 100*psi**2 + (v - vRef)**2 + 0.1*delta**2 + 0.1*a**2
    return 10*cte**2 + 100*epsi**2 + 1*(v - vRef)**2 + 0.1*delta**2 + 0.1*a**2


def main():
    ocp = AcadosOcp()
    ocp.model = carBicycleModel()

    Tf = 2.0
    N = 20
    ocp.dims.N = N

    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]
    ny = nx + nu

    ocp.cost.cost_type = "EXTERNAL"
    ocp.model.cost_expr_ext_cost = costFunc(ocp.model)
    # ocp.cost.cost_type = "LINEAR_LS"
    # Q = 2*np.diag([0, 1e1, 1e2, 1e0])
    # R = 2*np.diag([1e-1, 1e-1])
    # ocp.cost.W = scipy.linalg.block_diag(Q, R)

    # Vx = np.zeros((ny, nx))
    # Vx[:nx, :nx] = np.eye(nx)
    # ocp.cost.Vx = Vx

    # Vu = np.zeros((ny, nu))
    # Vu[nx, 0] = 1
    # Vu[nx+1, 0] = 1
    # ocp.cost.Vu = Vu

    # ocp.cost.yref = np.array([0, 0, 0, 4, 0, 0])


    aMax = 1
    deltaMax = 0.57
    ocp.constraints.constr_type = "BGH"
    ocp.constraints.lbu = np.array([-deltaMax, -aMax])
    ocp.constraints.ubu = np.array([deltaMax, aMax])
    ocp.constraints.idxbu = np.array([0, 1])

    x0 = np.array([-10, 0, 0, 0])
    ocp.constraints.x0 = x0

    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM" #"PARTIAL_CONDENSING_HPIPM" "FULL_CONDENSING_QPOASES"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver_iter_max = 1000

    ocp_solver = AcadosOcpSolver(ocp, 'acados_ocp_' + ocp.model.name + '.json')
    ocp_integrator = AcadosSimSolver(ocp, 'acados_ocp_' + ocp.model.name + '.json')
    
    Nsim = 100
    simX = np.ndarray((Nsim + 1, nx))
    simU = np.ndarray((Nsim, nu))

    x_cur = x0
    simX[0,:] = x0

    time_solve = 0
    for i in range(Nsim):

        ocp_solver.set(0, "lbx", x_cur)
        ocp_solver.set(0, "ubx", x_cur)

        start = time.time()
        solver_status = ocp_solver.solve()
        t = time.time() - start
        print(f'time: {t*1000:.2f}[ms], iter: {i}')
        time_solve += t

        if solver_status != 0:
            raise Exception(f'solver error: {solver_status}')

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
    
main()