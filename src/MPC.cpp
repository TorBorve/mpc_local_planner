#include "mpc_local_planner/MPC.h"
#include "mpc_local_planner/bounds.h"
#include "mpc_local_planner/utilities.h"
#include "mpc_local_planner/solverCppAD.h"
#include "mpc_local_planner/AcadosSolver.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>

#include <fstream>

namespace mpc
{
    MPC::MPC(const std::vector<Point> &track, size_t N, double dt, Bound steeringAngle, double maxSteeringRotationSpeed, double wheelbase) : track_{track}, N_{N}, dt_{dt}, steeringAngle_{steeringAngle}, maxSteeringRotationSpeed_{maxSteeringRotationSpeed},
                                                                                                                                             wheelbase_{wheelbase}, x_start_{0}, y_start_{N}, psi_start_{2 * N}, v_start_{3 * N}, cte_start_{4 * N}, epsi_start_{5 * N},
                                                                                                                                             delta_start_{6 * N}, a_start_{7 * N - 1} // -1 due to N-1 actuator variables
    {
        ros::NodeHandle nh;
        trackPub_ = nh.advertise<nav_msgs::Path>("global_path", 1);
        mpcPathPub_ = nh.advertise<nav_msgs::Path>("local_path", 1);
        polynomPub_ = nh.advertise<nav_msgs::Path>("interpolated_path", 1);
    }

    MPCReturn MPC::solve(const OptVariables &optVars)
    {
        const State &state = optVars.x;

        pubTf(state);

        double rotation;
        Eigen::Vector4d coeffs;
        bool forward;
        calcCoeffs(state, rotation, coeffs, forward);

        State transformedState{0, 0, rotation, state.vel, 0, 0};
        calcState(transformedState, coeffs, forward);
        OptVariables transformedOptVar{transformedState, optVars.u};
        if (optVars.x.vel < 1)
        {
            transformedOptVar.x.vel = 1;
        }
        auto result = solve(transformedOptVar, coeffs);

        double rotangle = state.psi - rotation;
        auto &x = result.mpcHorizon;
        for (unsigned int i = 0; i < x.size(); i++)
        {
            // rotate back
            double dx = x[i].x.x;
            double dy = x[i].x.y;
            x[i].x.x = dx * cos(rotangle) - dy * sin(rotangle);
            x[i].x.y = dx * sin(rotangle) + dy * cos(rotangle);

            // // shift coordinates
            x[i].x.x += state.x;
            x[i].x.y += state.y;
        }

        auto polyPath = getPathMsg(coeffs);
        auto &points = polyPath.poses;
        for (unsigned int i = 0; i < points.size(); i++)
        {
            double dx = points[i].pose.position.x;
            double dy = points[i].pose.position.y;
            points[i].pose.position.x = dx * cos(rotangle) - dy * sin(rotangle);
            points[i].pose.position.y = dx * sin(rotangle) + dy * cos(rotangle);

            points[i].pose.position.x += state.x;
            points[i].pose.position.y += state.y;
        }
        polynomPub_.publish(polyPath);
        trackPub_.publish(getPathMsg(track_));
        mpcPathPub_.publish(getPathMsg(result));

        return result;
    }

    MPCReturn MPC::solve(const OptVariables &optVars, const Eigen::Vector4d &coeffs)
    {
        switch (solver_)
        {
        case Solver::Acados:
            return solveAcados(optVars, coeffs);
            break;
        case Solver::CppAD:
            return solveCppAD(optVars, coeffs);
            break;
        default:
            std::string error = "invalid solver. solver_ variable not set correctly.";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error{error};
        }
    }

    MPCReturn MPC::toMPCReturn(const CppAD::ipopt::solve_result<Dvector> &solution, double time)
    {
        const auto &x = solution.x;
        MPCReturn ret;
        ret.mpcHorizon.resize(N_ - 1); // TODO should be N error due to a?
        ret.u0 = Input{x[a_start_], x[delta_start_]};
        for (int i = 0; i < N_ - 1; i++)
        { // TODO should be N error due to a?
            State state{x[x_start_ + i], x[y_start_ + i], x[psi_start_ + i], x[v_start_ + i],
                        x[cte_start_ + i], x[epsi_start_ + i]};
            Input input{x[a_start_ + i], x[delta_start_ + i]};
            ret.mpcHorizon[i] = OptVariables{state, input};
        }
        ret.cost = solution.obj_value;
        ret.success = solution.success;
        ret.computeTime = time;
        return ret;
    }

    void MPC::calcCoeffs(const State &state, double &rotation, Eigen::Vector4d &coeffs, bool &forward) const
    {
        size_t start, end;
        getTrackSection(start, end, state);

        double minCost = 1e19;
        for (double rot = -M_PI_2; rot < 0; rot += M_PI_2 / 3)
        {
            double curCost = minCost + 1;
            bool curForward = true;
            Eigen::Vector4d curCoeffs = interpolate(state, rot, start, end, curCost, curForward);
            if (curCost < minCost)
            {
                minCost = curCost;
                forward = curForward;
                coeffs = curCoeffs;
                rotation = rot;
            }
        }
        return;
    }

    void MPC::calcState(State &state, const Eigen::Vector4d &coeffs, bool &forward) const
    {
        state.cte = state.y - polyEval(state.x, coeffs);
        double dy = coeffs[1] + 2 * state.x * coeffs[2] + 3 * coeffs[3] * state.x * state.x;
        double dx = 1;
        if (dy > 100)
        {
            dy = 100;
        }
        else if (dy < -100)
        {
            dy = -100;
        }

        if (forward)
        {
            state.epsi = state.psi - atan2(dy, dx);
        }
        else
        {
            state.epsi = state.psi - atan2(-dy, -dx);
        }

        if (state.epsi > M_PI)
        {
            state.epsi -= 2 * M_PI;
        }
        else if (state.epsi < -M_PI)
        {
            state.epsi += 2 * M_PI;
        }
        return;
    }

    Eigen::Vector4d MPC::interpolate(const State &state, double rotation, size_t start, size_t end, double &cost, bool &forward) const
    {
        Eigen::VectorXd xVals(end - start);
        Eigen::VectorXd yVals(end - start);
        double angle = rotation - state.psi;

        for (unsigned int i = start; i < end; i++)
        {
            // shift points so that the state is in the origo
            double dx = track_[i].x - state.x;
            double dy = track_[i].y - state.y;

            // rotate points so that state.psi = 0 in the new refrence frame
            xVals[i - start] = dx * cos(angle) - dy * sin(angle);
            yVals[i - start] = dx * sin(angle) + dy * cos(angle);
        }

        auto coeffs = polyfit(xVals, yVals, 3);
        assert(coeffs.size() == 4);

        cost = 0;
        for (unsigned int i = 0; i < yVals.size(); i++)
        {
            cost += distSqrd(yVals[i] - polyEval(xVals[i], coeffs), 0);
        }

        if (xVals[0] <= xVals[xVals.size() - 1])
        {
            forward = true;
        }
        else
        {
            forward = false;
        }
        return coeffs;
    }

    void MPC::model(OptVariables &optVars, const Input &u)
    {
        model(optVars, u, this->dt_);
    }

    void MPC::model(OptVariables &optVars, const Input &u, double dt)
    {
        const double maxInc = maxSteeringRotationSpeed_ * dt_;
        State &state = optVars.x;
        double delta = u.delta;
        if (delta < optVars.u.delta - maxInc)
        {
            delta = optVars.u.delta - maxInc;
            ROS_WARN("Unable to turn wheels fast enough");
        }
        else if (delta > optVars.u.delta + maxInc)
        {
            delta = optVars.u.delta + maxInc;
            ROS_WARN("Unable to turn wheels fast enough");
        }
        optVars.u.delta = delta;
        state.x += state.vel * cos(state.psi) * dt;
        state.y += state.vel * sin(state.psi) * dt;
        state.psi += state.vel * tan(delta) / wheelbase_ * dt;
        state.vel += u.a * dt;
    }

    void MPC::getTrackSection(size_t &start, size_t &end, const State &state) const
    {
        double maxLen = 3;
        double minDistSqrd = distSqrd(state.x - track_[0].x, state.y - track_[0].y);
        size_t minIndex = 0;
        for (unsigned int i = 1; i < track_.size(); i++)
        {
            double curDistSqrd = distSqrd(state.x - track_[i].x, state.y - track_[i].y);
            if (curDistSqrd < minDistSqrd)
            {
                minDistSqrd = curDistSqrd;
                minIndex = i;
            }
        }

        double len = 0;
        size_t frontIndex = minIndex;
        size_t backIndex = minIndex;
        while (len < maxLen * maxLen && frontIndex < track_.size() - 1 && backIndex > 0)
        {
            frontIndex++;
            // backIndex--;
            len += distSqrd(track_[frontIndex].x - track_[frontIndex - 1].x, track_[frontIndex].y - track_[frontIndex - 1].y);
            // len += distSqrd(track_[backIndex].x - track_[backIndex + 1].x, track_[backIndex].y - track_[backIndex + 1].y);
        }
        start = backIndex;
        end = frontIndex;
        if (end - start < 4)
        {
            end = start + 4;
        }
        if (end >= track_.size())
        {
            start = 0;
            end = start + 4;
        }
        assert(end < track_.size());
    }

    void MPC::pubTf(const State &state) const
    {
        static tf2_ros::TransformBroadcaster br;

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "mpc_base_link";
        transformStamped.transform.translation.x = state.x;
        transformStamped.transform.translation.y = state.y;
        transformStamped.transform.translation.z = 0.5;
        tf2::Quaternion q;
        q.setRPY(0, 0, state.psi);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);
    }

    MPCReturn MPC::solveCppAD(const OptVariables &optVars, const Eigen::Vector4d &coeffs)
    {
        using Dvector = CPPAD_TESTVECTOR(double);
        const State &state = optVars.x;
        const double maxInc = maxSteeringRotationSpeed_ * dt_;

        auto start = std::chrono::high_resolution_clock::now();

        double x = state.x;
        double y = state.y;
        double psi = state.psi;
        double v = state.vel;
        double cte = state.cte;
        double epsi = state.epsi;
        double delta = optVars.u.delta;

        // TODO: Set the number of model variables (includes both states and inputs).
        // For example: If the state is a 4 element vector, the actuators is a 2
        // element vector and there are 10 timesteps. The number of variables is:
        //
        // 4 * 10 + 2 * 9
        size_t n_vars = N_ * 6 + (N_ - 1) * 2;
        // TODO: Set the number of constraints
        size_t n_constraints = N_ * 6 + N_ - 2;

        // Initial value of the independent variables.
        // SHOULD BE 0 besides initial state.
        Dvector vars(n_vars);
        for (unsigned int i = 0; i < n_vars; i++)
        {
            vars[i] = 0;
        }

        for (unsigned int i = 0; i < N_; i++)
        {
            vars[x_start_ + i] = x;
            vars[y_start_ + i] = y;
            vars[psi_start_ + i] = psi;
            vars[v_start_ + i] = v;
            vars[cte_start_ + i] = cte;
            vars[epsi_start_ + i] = epsi;
        }

        BoundVector varBounds(n_vars, Bound::noBound());

        // upper/lower limits for delta set to -25/25
        // degrees(values in radians)
        for (unsigned int i = delta_start_; i < a_start_; i++)
        {
            varBounds[i] = steeringAngle_;
        }

        // acceleration/deceleration upper/lower limits
        for (unsigned int i = a_start_; i < n_vars; i++)
        {
            varBounds[i] = Bound{-1, 1};
        }

        // Lower and upper limits for the constraints
        // Should be 0 besides initial state.
        BoundVector constraintBounds(n_constraints, Bound::zeroBound());

        constraintBounds[x_start_] = Bound{x, x};
        constraintBounds[y_start_] = Bound{y, y};
        constraintBounds[psi_start_] = Bound{psi, psi};
        constraintBounds[v_start_] = Bound{v, v};
        constraintBounds[cte_start_] = Bound{cte, cte};
        constraintBounds[epsi_start_] = Bound{epsi, epsi};

        for (unsigned int i = delta_start_; i < delta_start_ + N_ - 2; i++)
        {
            constraintBounds[i] = Bound{-maxInc, maxInc};
        }

        Bound deltaBound = steeringAngle_;
        if ((delta - maxInc) > deltaBound.lower)
        {
            deltaBound.lower = delta - maxInc;
        }
        if ((delta + maxInc) < deltaBound.upper)
        {
            deltaBound.upper = delta + maxInc;
        }

        // object that computes objective and constraints
        FG_eval fg_eval(coeffs, N_, dt_, wheelbase_);

        //
        // NOTE: You don't have to worry about these options
        //
        // options for IPOPT solver
        std::string options;
        // Uncomment this if you'd like more print information
        options += "Integer print_level  0\n";
        // NOTE: Setting sparse to true allows the solver to take advantage
        // of sparse routines, this makes the computation MUCH FASTER. If you
        // can uncomment 1 of these and see if it makes a difference or not but
        // if you uncomment both the computation time should go up in orders of
        // magnitude.
        options += "Sparse  true        forward\n";
        options += "Sparse  true        reverse\n";
        // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
        // Change this as you see fit.
        options += "Numeric max_cpu_time          10.0\n";

        // place to return solution
        CppAD::ipopt::solve_result<Dvector> solution;

        // solve the problem
        Dvector varsLower = toCppAD(getLower(varBounds));
        Dvector varsUpper = toCppAD(getUpper(varBounds));
        Dvector constraintsLower = toCppAD(getLower(constraintBounds));
        Dvector constraintsUpper = toCppAD(getUpper(constraintBounds));
        CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, varsLower, varsUpper, constraintsLower,
            constraintsUpper, fg_eval, solution);

        if (!solution.status == CppAD::ipopt::solve_result<Dvector>::success)
        {
            std::cout << "Error: Failed to solve nlp\n";
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        return toMPCReturn(solution, duration.count());
    }

    MPCReturn MPC::solveAcados(const OptVariables &optVars, const Eigen::Vector4d &coeffs)
    {
        static AcadosSolver solver{optVars};

        int N = BICYCLE_MODEL_N;
        auto start = std::chrono::high_resolution_clock::now();

        // initial condition
        int idxbx0[NBX0];
        idxbx0[0] = 0;
        idxbx0[1] = 1;
        idxbx0[2] = 2;
        idxbx0[3] = 3;
        idxbx0[4] = 4;

        double lbx0[NBX0];
        double ubx0[NBX0];
        lbx0[0] = optVars.x.x;
        ubx0[0] = optVars.x.x;
        lbx0[1] = optVars.x.y;
        ubx0[1] = optVars.x.y;
        lbx0[2] = optVars.x.psi;
        ubx0[2] = optVars.x.psi;
        lbx0[3] = optVars.x.vel;
        ubx0[3] = optVars.x.vel;
        lbx0[4] = optVars.u.delta;
        ubx0[4] = optVars.u.delta;

        ocp_nlp_constraints_model_set(solver.config, solver.dims, solver.in, 0, "idxbx", idxbx0);
        ocp_nlp_constraints_model_set(solver.config, solver.dims, solver.in, 0, "lbx", lbx0);
        ocp_nlp_constraints_model_set(solver.config, solver.dims, solver.in, 0, "ubx", ubx0);

        // set parameters
        double p[NP];
        p[0] = coeffs[0];
        p[1] = coeffs[1];
        p[2] = coeffs[2];
        p[3] = coeffs[3];

        for (int ii = 0; ii <= N; ii++)
        {
            bicycle_model_acados_update_params(solver.capsule, ii, p, NP);
        }

        // prepare evaluation
        int NTIMINGS = 1;
        double min_time = 1e12;
        double kkt_norm_inf;
        double elapsed_time;
        int sqp_iter;

        double xtraj[NX * (N + 1)];
        double utraj[NU * N];

        // solve ocp in loop
        int rti_phase = 0;
        int status;

        for (int ii = 0; ii < NTIMINGS; ii++)
        {
            ocp_nlp_solver_opts_set(solver.config, solver.opts, "rti_phase", &rti_phase);
            status = bicycle_model_acados_solve(solver.capsule);
            ocp_nlp_get(solver.config, solver.solver, "time_tot", &elapsed_time);
            min_time = MIN(elapsed_time, min_time);
        }

        // get the solution
        for (int ii = 0; ii <= solver.dims->N; ii++)
            ocp_nlp_out_get(solver.config, solver.dims, solver.out, ii, "x", &xtraj[ii * NX]);
        for (int ii = 0; ii < solver.dims->N; ii++)
            ocp_nlp_out_get(solver.config, solver.dims, solver.out, ii, "u", &utraj[ii * NU]);

        if (status != ACADOS_SUCCESS)
        {
            ROS_ERROR("bicycle_model_acados_solve() failed with status %d.\n", status);
            solver.reInit(optVars);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        MPCReturn ret;
        ret.mpcHorizon.resize(N - 1);
        ret.u0 = Input{utraj[1], utraj[0]};
        for (int i = 0; i < N - 1; i++)
        {
            State state{xtraj[0 + NX * i], xtraj[1 + NX * i], xtraj[2 + NX * i], xtraj[3 + NX * i], 0, 0};
            Input input{utraj[1 + NU * i], utraj[0 + NU * i]};
            ret.mpcHorizon.at(i) = OptVariables{state, input};
        }
        ret.cost = 0;
        ret.success = true;
        ret.computeTime = duration.count();
        return ret;
    }
}