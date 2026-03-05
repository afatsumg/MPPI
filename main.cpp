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

    // 2. Parking reference (target: X=20m, Y=10m, Psi=0 rad, speed=0 m/s)
    // State vector order: [x, y, psi, beta, vx, vy, r, delta, Fx]
    Eigen::VectorXd reference = Eigen::VectorXd::Zero(9);
    reference(0) = 5.0; // target X
    reference(1) = 3.0; // target Y
    reference(2) = 0.0;  // target orientation (straight parking)
    reference(4) = 0.0;  // target speed (we want to stop)

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
        
        // MPPI chooses the best parking trajectory from 1000 scenarios each step
        Eigen::Vector2d u_optimal = mppi.computeControl(x, reference);

        // Modeli ilerlet
        Eigen::VectorXd x_next(9);
        car.Predict(x, u_optimal, x_next);
        x = x_next;
        // accumulate cost for tuning
        total_cost += mppi.computeRunningCost(x, reference);
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
        double dist = std::sqrt(std::pow(x(0)-reference(0),2) + std::pow(x(1)-reference(1),2));
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