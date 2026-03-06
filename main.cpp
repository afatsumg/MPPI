#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include "model.h"
#include "MPPIController.h"

int main() {
    VehicleModel car;
    MPPIController mppi(&car);

    // optionally read parameters from config file
    if (!mppi.loadConfig("mppi.cfg")) {
        std::cerr << "warning: mppi.cfg could not be read, using defaults\n";
    }

    // two parked obstacles (x, y, psi, safety radius)
    std::vector<Eigen::Vector4d> obstacles;
    obstacles.emplace_back(3.0, 7.0, 0.0, 1.0);   // car parked on left
    obstacles.emplace_back(9.0, 7.0, 0.0, 1.0);   // car parked on right
    mppi.setObstacles(obstacles);

    // write obstacles to a CSV
    std::ofstream obs_csv("obstacles.csv");
    if (obs_csv.is_open()) {
        obs_csv << "x,y,psi,r\n";
        for (auto &o : obstacles)
            obs_csv << o(0) << "," << o(1) << "," << o(2) << "," << o(3) << "\n";
        obs_csv.close();
    }

    // 1. Initial state (we are stopped on the roadside)
    Eigen::VectorXd x = Eigen::VectorXd::Zero(9);
    x(4) = 2.0; // starting at very low speed (2 m/s)

    // 2. Parking reference (goal placed between the two parked cars)
    // State vector order: [x, y, psi, beta, vx, vy, r, delta, Fx]
    Eigen::VectorXd goal = Eigen::VectorXd::Zero(9);
    goal(0) = 6.0; // target X, placed between the two parked vehicles
    goal(1) = 6.5; // target Y (slightly in front of them, to make it reachable)
    goal(2) = 0.0;  // target orientation (straight parking)
    goal(4) = 0.0;  // target speed (we want to stop)

    // Build a smooth cubic (Hermite) trajectory from current pose to goal.
    // We define tangents based on initial heading and a stopped final heading.
    auto buildReferenceTrajectory = [&](const Eigen::VectorXd &state) {
        std::vector<Eigen::VectorXd> traj;
        traj.reserve(mppi.N);

        Eigen::Vector2d p0(state(0), state(1));
        Eigen::Vector2d p1(goal(0), goal(1));

        // Starting tangent aligned with initial heading (psi=0 -> along +X)
        Eigen::Vector2d t0(1.0, 0.0);
        // End tangent is zero (we want to stop and face forward)
        Eigen::Vector2d t1(0.0, 0.0);

        // scale tangents to create a reasonable path length
        double scale = (p1 - p0).norm();
        t0 *= scale;

        for (int i = 0; i < mppi.N; ++i) {
            double s = double(i + 1) / double(mppi.N); // 0..1
            double h00 = 2*s*s*s - 3*s*s + 1;
            double h10 = s*s*s - 2*s*s + s;
            double h01 = -2*s*s*s + 3*s*s;
            double h11 = s*s*s - s*s;

            Eigen::Vector2d pos = h00 * p0 + h10 * t0 + h01 * p1 + h11 * t1;

            Eigen::VectorXd ref = goal;
            ref(0) = pos.x();
            ref(1) = pos.y();

            // smooth speed profile (slowing down toward the end)
            ref(4) = (1.0 - s) * 2.0;
            traj.push_back(ref);
        }

        return traj;
    };

    // export the planned reference trajectory for plotting
    std::ofstream ref_csv("reference.csv");
    if (ref_csv.is_open()) {
        ref_csv << "x,y\n";
        auto initial_traj = buildReferenceTrajectory(x);
        for (auto &r : initial_traj) {
            ref_csv << r(0) << "," << r(1) << "\n";
        }
        ref_csv.close();
    }

    std::cout << "Time\tX\tY\tPsi\tVx\tSteer\tFx" << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;

    // open csv file and write header
    std::ofstream csv("states.csv");
    if (csv.is_open()) {
        csv << "time,x,y,psi,beta,vx,vy,r,delta,Fx\n";
    } else {
        std::cerr << "Warning: states.csv could not be opened\n";
    }

    // 10-second parking maneuver
    double total_cost = 0.0;
    for (double t = 0; t <= 10.0; t += car.dt) {
        // build a simple reference trajectory toward the goal
        auto reference_traj = buildReferenceTrajectory(x);

        // MPPI chooses the best trajectory based on the reference trajectory
        Eigen::Vector2d u_optimal = mppi.computeControl(x, reference_traj);

        // step the model forward
        Eigen::VectorXd x_next(9);
        car.Predict(x, u_optimal, x_next);
        x = x_next;
        // accumulate cost for tuning (use first point in trajectory as current desired)
        total_cost += mppi.computeRunningCost(x, reference_traj.front());
        // check for NaN or overflow states
        if (x.hasNaN()) {
            std::cerr << "State became NaN, aborting simulation at t=" << t << "\n";
            break;
        }
        if (x.head<2>().array().abs().maxCoeff() > 100.0) {
            std::cerr << "State exploded (x or y >100), aborting at t=" << t << "\n";
            break;
        }

        // print state every 0.5 seconds
        if (std::fmod(t, 0.5) < 0.001) {
            std::cout << t << "\t" << x(0) << "\t" << x(1) << "\t" 
                      << x(2) << "\t" << x(4) << "\t" 
                      << u_optimal(0) << "\t" << u_optimal(1) << std::endl;
        }

        // write state to states.csv at each step
        if (csv.is_open()) {
            csv << t << ','
                << x(0) << ',' << x(1) << ',' << x(2) << ','
                << x(3) << ',' << x(4) << ',' << x(5) << ','
                << x(6) << ',' << x(7) << ',' << x(8) << '\n';
        }

        // end simulation if close enough to goal
        double dist = std::sqrt(std::pow(x(0)-goal(0),2) + std::pow(x(1)-goal(1),2));
        //std::cout << "Distance to target: " << dist << " m" << std::endl;
        //std::cout << "Velocity: " << x(4) << " m/s" << std::endl;
        if (std::isnan(x(4))) {
            std::cerr << "Warning: velocity became NaN, stopping simulation and resetting value\n";
            x(4) = 0.0;
        }
        if (dist < 0.1 && x(4) < 0.1) {
            std::cout << dist << " m away, speed " << x(4) << " m/s, parking completed.\n";
            std::cout << "Parking complete! T: " << t << std::endl;
            break;
        }
    }
    std::cout << "total_cost=" << total_cost << std::endl;

    if (csv.is_open()) csv.close();
    return 0;
}