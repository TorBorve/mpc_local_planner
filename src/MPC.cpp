#include "mpc_local_planner/MPC.h"
#include "mpc_local_planner/bounds.h"
#include "mpc_local_planner/utilities.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <fstream>

namespace mpc {

    class FG_eval {
    public:
        using ADvector = CPPAD_TESTVECTOR(CppAD::AD<double>);
        
        FG_eval(const Eigen::Vector4d& coeffs, size_t N, double dt) :
            coeffs{coeffs}, N{N}, dt{dt}, x_start{0}, y_start{N}, 
            psi_start{2*N}, v_start{3*N}, cte_start{4*N}, epsi_start{5*N}, 
            delta_start{6*N}, a_start{7*N - 1} 
        {

        }
        
        void operator()(ADvector& fg, const ADvector& vars) {
            // cost function
            fg[0] = 0;

            for (unsigned int i=0; i<N; i++)
            {
            fg[0] += 500*CppAD::pow(vars[cte_start+i]-ref_cte, 2);
            fg[0] += 2000*CppAD::pow(vars[epsi_start+i]-ref_epsi, 2);
            // fg[0] += 1*CppAD::pow(vars[v_start+i]-ref_v, 2);
            }

            // minimize the use of actuators
            for (unsigned int i=0; i<N-1; i++)
            {
            fg[0] += 25*CppAD::pow(vars[delta_start+i], 2);
            // fg[0] += 25*CppAD::pow(vars[a_start+i], 2);
            //fg[0] += 700*CppAD::pow(vars[delta_start + i] * vars[v_start+i], 2);
            }

            // minimize the value gap between sequential actuations
            for (unsigned int i=0; i<N-2; i++)
            {
            fg[0] += 200*CppAD::pow(vars[delta_start+i+1] - vars[delta_start+i], 2); 
            // fg[0] += 20*CppAD::pow(vars[a_start+i+1] - vars[a_start+i], 2);
            }
    
            //
            // Setup Constraints
            //
            // NOTE: In this section you'll setup the model constraints.

            // Initial constraints
            //
            // We add 1 to each of the starting indices due to cost being located at
            // index 0 of `fg`.
            // This bumps up the position of all the other values.
            fg[1 + x_start] = vars[x_start];
            fg[1 + y_start] = vars[y_start];
            fg[1 + psi_start] = vars[psi_start];
            fg[1 + v_start] = vars[v_start];
            fg[1 + cte_start] = vars[cte_start];
            fg[1 + epsi_start] = vars[epsi_start];

            // The rest of the constraints
            for (unsigned int i = 0; i < N -1; i++) 
            {
            // The state at time t+1 .
            CppAD::AD<double> x1 = vars[x_start + i + 1];
            CppAD::AD<double> y1 = vars[y_start + i + 1];
            CppAD::AD<double> psi1 = vars[psi_start + i + 1];
            CppAD::AD<double> v1 = vars[v_start + i + 1];
            CppAD::AD<double> cte1 = vars[cte_start + i + 1];
            CppAD::AD<double> epsi1 = vars[epsi_start + i + 1];

            // The state at time t.
            CppAD::AD<double> x0 = vars[x_start + i];
            CppAD::AD<double> y0 = vars[y_start + i];
            CppAD::AD<double> psi0 = vars[psi_start + i];
            CppAD::AD<double> v0 = vars[v_start + i];
            CppAD::AD<double> cte0 = vars[cte_start + i];
            CppAD::AD<double> epsi0 = vars[epsi_start + i];
            // Only consider the actuation at time t.
            CppAD::AD<double> delta0 = vars[delta_start + i];
            CppAD::AD<double> a0 = vars[a_start + i];
            //if (i > 0) {   // use previous actuations (to account for latency)
            //  a0 = vars[a_start + i - 1];
            //  delta0 = vars[delta_start + i - 1];
            //}
            
            CppAD::AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
            CppAD::AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0*x0 + 2*coeffs[2]*x0 + coeffs[1]);

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // Recall the equations for the model:
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
            fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + psi_start + i] = psi1 - (psi0 + v0 * CppAD::tan(delta0) / Lf * dt);
            fg[2 + v_start + i] = v1 - v0; //v1 - (v0 + a0 * dt);
            fg[2 + cte_start + i] =
                cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + epsi_start + i] =
                epsi1 - ((psi0 - psides0) + v0 * CppAD::tan(delta0) / Lf * dt);
            }

            for (unsigned int i = delta_start; i < delta_start + N - 2; i++) {
                CppAD::AD<double> delta0 = vars[i];
                CppAD::AD<double> delta1 = vars[i + 1];
                fg[1 + i] = delta1 - delta0;
            }
            return;
        }
    private:
        Eigen::Vector4d coeffs;
        const size_t N;
        const double dt;

        const size_t x_start;
        const size_t y_start;
        const size_t psi_start;
        const size_t v_start;
        const size_t cte_start;
        const size_t epsi_start;
        const size_t delta_start;
        const size_t a_start;

        const double Lf = 2.67;
        const double ref_cte = 0;
        const double ref_epsi = 0;
        const double ref_v = 5; 
    };

    MPC::MPC(const std::vector<Point>& track, size_t N, double dt) : 
            track_{track}, N{N}, dt{dt}, x_start{0}, y_start{N},
            psi_start{2*N}, v_start{3*N}, cte_start{4*N}, epsi_start{5*N},
            delta_start{6*N}, a_start{7*N - 1} // -1 due to N-1 actuator variables
    {
        ros::NodeHandle nh;
        trackPub_ = nh.advertise<nav_msgs::Path>("global_path", 1);
        mpcPathPub_ = nh.advertise<nav_msgs::Path>("local_path", 1);
        polynomPub_ = nh.advertise<nav_msgs::Path>("interpolated_path", 1);
    }

    MPCReturn MPC::solve(const OptVariables& optVars) {
        static tf2_ros::TransformBroadcaster br;
        const State& state = optVars.x;

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

        double rotation;
        Eigen::Vector4d coeffs;
        calcCoeffs(state, rotation, coeffs);

        State transformedState{0, 0, rotation, state.vel, 0, 0};
        calcState(transformedState, coeffs);
        OptVariables transformedOptVar{transformedState, optVars.u};
        if(optVars.x.vel < 1) {
            transformedOptVar.x.vel = 1;
        }
        auto result = solve(transformedOptVar, coeffs);

        double rotangle = state.psi - rotation;
        auto& x = result.mpcHorizon;
        for (unsigned int i = 0; i < x.size(); i++) {
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
        auto& points = polyPath.poses;
        for (unsigned int i = 0; i < points.size(); i++) {
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

    MPCReturn MPC::solve(const OptVariables& optVars, const Eigen::Vector4d& coeffs){
        using Dvector = CPPAD_TESTVECTOR(double);
        const State& state = optVars.x;

        // static double prevDelta = 0;
        const double maxInc = 800.0 / 17.3 * M_PI / 180.0 * dt;

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
        size_t n_vars = N * 6 + (N-1)*2;
        // TODO: Set the number of constraints
        size_t n_constraints = N * 6 + N - 2;


        // Initial value of the independent variables.
        // SHOULD BE 0 besides initial state.
        Dvector vars(n_vars);
        for (unsigned int i = 0; i < n_vars; i++) {
            vars[i] = 0;
        }

        for (unsigned int i = 0; i < N; i++) {
            vars[x_start + i] = x;
            vars[y_start + i] = y;
            vars[psi_start + i] = psi;
            vars[v_start + i] = v;
            vars[cte_start + i] = cte;
            vars[epsi_start + i] = epsi;
        }

        BoundVector varBounds(n_vars, Bound::noBound());

        // upper/lower limits for delta set to -25/25
        // degrees(values in radians)
        for (unsigned int i = delta_start; i < a_start; i++) {
            varBounds[i] = Bound{-0.436332*Lf, 0.436332*Lf};
        }

        // acceleration/deceleration upper/lower limits 
        for (unsigned int i = a_start; i < n_vars; i++) {
            varBounds[i] = Bound{0, 0};
        }


        // Lower and upper limits for the constraints
        // Should be 0 besides initial state.
        BoundVector constraintBounds(n_constraints, Bound::zeroBound());

        constraintBounds[x_start] = Bound{x, x};
        constraintBounds[y_start] = Bound{y, y};
        constraintBounds[psi_start] = Bound{psi, psi};
        constraintBounds[v_start] = Bound{v, v};
        constraintBounds[cte_start] = Bound{cte, cte};
        constraintBounds[epsi_start] = Bound{epsi, epsi};

        for (unsigned int i = delta_start; i < delta_start + N - 2; i++) {
            constraintBounds[i] = Bound{-maxInc, maxInc};
        }
        
        Bound deltaBound{-0.57, 0.57};
        if ((delta - maxInc) > deltaBound.lower) {
            deltaBound.lower = delta - maxInc;
        }
        if ((delta + maxInc) < deltaBound.upper) {
            deltaBound.upper = delta + maxInc;
        }
        varBounds[delta_start] = deltaBound;
        // ROS_WARN("l: %f, u: %f", deltaBound.lower, deltaBound.upper);

        // object that computes objective and constraints
        FG_eval fg_eval(coeffs, N, dt);

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

        if (!solution.status == CppAD::ipopt::solve_result<Dvector>::success){
            std::cout << "Error: Failed to solve nlp\n";
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        // double delta = solution.x[delta_start];
        // if (delta < prevDelta - maxInc) {
        //     delta = prevDelta - maxInc;
        //     ROS_WARN("Unable to turn wheels fast enough, MPC");
        // } else if (delta > prevDelta + maxInc) {
        //     delta = prevDelta + maxInc;
        //     ROS_WARN("Unable to turn wheels fast enough, MPC");
        // }
        // prevDelta = delta;
        return toMPCReturn(solution, duration.count());
    }

    MPCReturn MPC::toMPCReturn(const CppAD::ipopt::solve_result<Dvector>& solution, double time){
        const auto& x = solution.x;
        MPCReturn ret;
        ret.mpcHorizon.resize(N - 1); // TODO should be N error due to a?
        ret.u0 = Input{x[a_start], x[delta_start]};
        for (int i = 0; i < N - 1; i++){ // TODO should be N error due to a?
            State state{x[x_start + i], x[y_start + i], x[psi_start + i], x[v_start + i],
                        x[cte_start + i], x[epsi_start + i]};
            Input input{x[a_start + i], x[delta_start + i]};
            ret.mpcHorizon[i] = OptVariables{state, input};
        }
        ret.cost = solution.obj_value;
        ret.success = solution.success;
        ret.computeTime = time;
        return ret;
    }

    void MPC::calcCoeffs(const State& state, double& rotation, Eigen::Vector4d& coeffs) const {
        size_t start, end;
        getTrackSection(start, end, state);

        double minCost = 1e19;
        for (double rot = -M_PI_2; rot < M_PI_2; rot += M_PI / 10.0) {
            double curCost = minCost + 1;
            Eigen::Vector4d curCoeffs = interpolate(state, rot, start, end, curCost);
            if (curCost < minCost) {
                minCost = curCost;
                coeffs = curCoeffs;
                rotation = rot;
            }
        }
        // rotation = 0;
        // coeffs = interpolate(state, rotation, start, end, minCost);
        return;
    }

    void MPC::calcState(State& state, const Eigen::Vector4d& coeffs) const {
        state.cte = state.y - polyEval(state.x, coeffs);
        state.epsi = state.psi - atan(coeffs[1] + 2 * state.x * coeffs[2] + 3 * coeffs[3] * state.x * state.x);
        return;
    }

    Eigen::Vector4d MPC::interpolate(const State& state, double rotation, size_t start, size_t end, double& cost) const {
        Eigen::VectorXd xVals(end - start);
        Eigen::VectorXd yVals(end - start);
        double angle = -state.psi + rotation;

        for (unsigned int i = start; i < end; i++) {
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
        for (unsigned int i = 0; i < yVals.size(); i++) {
            cost += distSqrd(yVals[i] - polyEval(xVals[i], coeffs), 0);
        }
        return coeffs;
    }

    void MPC::model(OptVariables& optVars, const Input& u) {
        constexpr double dt = 0.1;
        constexpr double maxInc = 800.0 / 17.3 * M_PI / 180.0 * dt;
        State& state = optVars.x;
        double delta = u.delta;
        if (delta < optVars.u.delta - maxInc) {
            delta = optVars.u.delta - maxInc;
            ROS_WARN("Unable to turn wheels fast enough");
        } else if (delta > optVars.u.delta + maxInc) {
            delta = optVars.u.delta + maxInc;
            ROS_WARN("Unable to turn wheels fast enough");
        }
        optVars.u.delta = delta;
        state.x += state.vel * cos(state.psi) * dt;
        state.y += state.vel * sin(state.psi) * dt;
        state.psi += state.vel * tan(delta) / Lf * dt;
        state.vel += u.a * dt;
        // state.vel = 5;
    }

    void MPC::getTrackSection(size_t& start, size_t& end, const State& state) const {
        double maxLen = 3;
        double minDistSqrd = distSqrd(state.x - track_[0].x, state.y - track_[0].y);
        size_t minIndex = 0;
        for (unsigned int i = 1; i < track_.size(); i++) {
            double curDistSqrd = distSqrd(state.x - track_[i].x, state.y - track_[i].y);
            if (curDistSqrd < minDistSqrd) {
                minDistSqrd = curDistSqrd;
                minIndex = i;
            }
        }

        double len = 0;
        // bool forward = track_[minIndex + 1].x - track_[minIndex].x > 0;
        size_t frontIndex = minIndex;
        size_t backIndex = minIndex;
        while(len < maxLen * maxLen && frontIndex < track_.size() - 1 && backIndex > 0) {
            frontIndex++;
            // backIndex--;
            len += distSqrd(track_[frontIndex].x - track_[frontIndex - 1].x, track_[frontIndex].y - track_[frontIndex - 1].y);
            // len += distSqrd(track_[backIndex].x - track_[backIndex + 1].x, track_[backIndex].y - track_[backIndex + 1].y);
        }
        start = backIndex;
        end = frontIndex;  
        if (end - start < 4) {
            end = start + 4;
        }
        if (end >= track_.size()) {
            start = 0;
            end = start + 4;
        }
        assert(end < track_.size());      
    }
}