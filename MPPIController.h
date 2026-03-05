#ifndef MPPI_CONTROLLER_H
#define MPPI_CONTROLLER_H

#include <Eigen/Dense>
#include <vector>
#include "model.h"

class MPPIController {
public:
    int K = 1000; // number of samples
    int N = 30;   // horizon length
    double lambda = 1.0;
    Eigen::Matrix2d Sigma; // control noise covariance
    double pos_cost_weight = 100.0;
    double vel_cost_weight = 10.0;
    double delta_cost_weight = 50.0;
    double collision_cost_weight = 1e6;
    double obstacle_eps = 0.5;

    // physical limits
    double max_delta = 0.5;   // ±rad
    double max_Fx = 5000.0;   // ±N

    Eigen::MatrixXd U; // control sequence of size [2 x N]
    VehicleModel* model;

    MPPIController(VehicleModel* m) : model(m) {
        // default parameters
        U = Eigen::MatrixXd::Zero(2, N);
        Sigma << 0.05, 0, 0, 100.0; // noise scale for steer and Fx
    }

    // Read values from a simple key-value config file (one per line)
    // File might look like:
    //   # comment line
    //   K 1000
    //   N 30
    //   lambda 1.0
    //   Sigma00 0.05
    //   Sigma11 100.0
    bool loadConfig(const std::string &filename);

    // Additional obstacles (x, y, psi, safety_radius)
    std::vector<Eigen::Vector4d> obstacles;
    void setObstacles(const std::vector<Eigen::Vector4d>& obs) { obstacles = obs; }

    // Main loop from Algorithm 1
    Eigen::Vector2d computeControl(const Eigen::VectorXd& x_init, const Eigen::VectorXd& reference);
    // running cost is public (for tuner)
    double computeRunningCost(const Eigen::VectorXd& x, const Eigen::VectorXd& ref);
};

#endif