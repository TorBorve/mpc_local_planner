#include "mpc_local_planner/CppADMPC.h"

#include <iostream>
#include <thread>
#include <chrono>

int main(){
    mpc::State refState{4, 4, 0};
    mpc::State state{0, 0, 0};
    mpc::nonlinear::MPC mpc{refState};
    std::cout << "x\ty\tpsi\n";
    auto refresh = std::chrono::high_resolution_clock::now();
    while(true){
        auto solution = mpc.solve(state);
        mpc::nonlinear::diffDriveModel(state, solution.u0, 0.1);
        std::cout << state.x << '\t' << state.y << '\t' << state.psi << std::endl;
        refresh += std::chrono::milliseconds{200};
        std::this_thread::sleep_until(refresh);
    }

    return 0;
}