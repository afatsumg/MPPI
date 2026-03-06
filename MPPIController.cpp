#include "MPPIController.h"
#include <random>

#include <iostream>
#include <fstream>
#include <sstream>

bool MPPIController::loadConfig(const std::string &filename)
{
    std::ifstream in(filename);
    if (!in.is_open())
        return false;

    std::string line;
    while (std::getline(in, line))
    {
        // skip blank lines and # comments
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream iss(line);
        std::string key;
        double value;
        iss >> key >> value;
        if (key == "K")
            K = static_cast<int>(value);
        else if (key == "N")
            N = static_cast<int>(value);
        else if (key == "lambda")
            lambda = value;
        else if (key == "Sigma00")
            Sigma(0, 0) = value;
        else if (key == "Sigma11")
            Sigma(1, 1) = value;
        else if (key == "pos_cost_weight")
            pos_cost_weight = value;
        else if (key == "vel_cost_weight")
            vel_cost_weight = value;
        else if (key == "delta_cost_weight")
            delta_cost_weight = value;
        else if (key == "collision_cost_weight")
            collision_cost_weight = value;
        else if (key == "terminal_cost_weight")
            terminal_cost_weight = value;
        else if (key == "terminal_zone_radius")
            terminal_zone_radius = value;
        else if (key == "obstacle_eps")
            obstacle_eps = value;
        else if (key == "max_delta")
            max_delta = value;
        else if (key == "max_Fx")
            max_Fx = value;
    }

    // resize control sequence
    U = Eigen::MatrixXd::Zero(2, N);
    return true;
}

Eigen::Vector2d MPPIController::computeControl(const Eigen::VectorXd &x_init, const Eigen::VectorXd &reference)
{
    // create a constant reference trajectory and delegate
    std::vector<Eigen::VectorXd> traj(N, reference);
    return computeControl(x_init, traj);
}

Eigen::Vector2d MPPIController::computeControl(const Eigen::VectorXd &x_init, const std::vector<Eigen::VectorXd> &reference_traj)
{
    // reset (resize U if N or K changed)
    if (U.cols() != N)
    {
        U = Eigen::MatrixXd::Zero(2, N);
    }

    std::vector<Eigen::MatrixXd> delta_U(K, Eigen::MatrixXd(2, N));
    Eigen::VectorXd S = Eigen::VectorXd::Zero(K); // total costs

    std::default_random_engine generator;
    std::normal_distribution<double> dist(0.0, 1.0);

    // 1. Sampling and simulation (Steps 8-13)
    for (int k = 0; k < K; ++k)
    {
        Eigen::VectorXd x_sim = x_init;
        double cost_k = 0;

        for (int i = 0; i < N; ++i)
        {
            // generate random noise (delta u)
            Eigen::Vector2d du;
            du << dist(generator) * std::sqrt(Sigma(0, 0)),
                dist(generator) * std::sqrt(Sigma(1, 1));
            delta_U[k].col(i) = du;

            // apply control (u_i + delta_u_i) and clamp
            Eigen::Vector2d u_noisy = U.col(i) + du;
            u_noisy(0) = std::clamp(u_noisy(0), -max_delta, max_delta);
            u_noisy(1) = std::clamp(u_noisy(1), -max_Fx, max_Fx);

            Eigen::VectorXd x_next(9);
            model->Predict(x_sim, u_noisy, x_next);
            // if state blows up
            if (x_next.array().abs().maxCoeff() > 1e6)
            {
                cost_k = 1e12;
                break;
            }
            x_sim = x_next;

            // compute cost (how far from reference?)
            const Eigen::VectorXd &ref = (i < (int)reference_traj.size() ? reference_traj[i] : reference_traj.back());
            bool terminal = (i == N - 1);
            cost_k += computeRunningCost(x_sim, ref, terminal);

            // if x_sim became NaN then something has gone wrong
            if (std::isnan(x_sim.norm()))
            {
                std::cerr << "Warning: x_sim became NaN at sample " << k << " step " << i << "\n";
                cost_k = 1e12; // give a huge penalty and kill the scenario
                break;
            }
        }
        S(k) = cost_k;
    }

    // 2. Weighting and update (Steps 14-15)
    double min_S = S.minCoeff(); // for numerical stability
    double weight_sum = 0;
    Eigen::VectorXd weights(K);

    for (int k = 0; k < K; ++k)
    {
        weights(k) = std::exp(-(1.0 / lambda) * (S(k) - min_S));
        weight_sum += weights(k);
    }

    // prevent division by near-zero weight_sum
    if (weight_sum <= 0)
        weight_sum = 1e-9;

    // update the control sequence
    for (int i = 0; i < N; ++i)
    {
        Eigen::Vector2d weighted_du = Eigen::Vector2d::Zero();
        for (int k = 0; k < K; ++k)
        {
            weighted_du += weights(k) * delta_U[k].col(i);
        }
        U.col(i) += weighted_du / weight_sum;

        // NaN protection: if a column contains NaN, reset it to zero
        if (std::isnan(U.col(i).norm()))
        {
            std::cerr << "U became NaN on horizon index " << i << "; resetting to zero\n";
            U.col(i).setZero();
        }
    }

    // send first command and shift the sequence (Steps 16-18)
    Eigen::Vector2d final_u = U.col(0);
    // clamp
    final_u(0) = std::clamp(final_u(0), -max_delta, max_delta);
    final_u(1) = std::clamp(final_u(1), -max_Fx, max_Fx);
    if (std::isnan(final_u.norm()))
    {
        std::cerr << "final_u NaN detected, resetting all controls\n";
        final_u.setZero();
        U.setZero();
    }
    for (int i = 0; i < N - 1; ++i)
        U.col(i) = U.col(i + 1);
    U.col(N - 1) = Eigen::Vector2d::Zero(); // u_init

    return final_u;
}

// in MPPIController.cpp
double MPPIController::computeRunningCost(const Eigen::VectorXd &x,
                                          const Eigen::VectorXd &ref,
                                          bool terminal)
{
    if (x.hasNaN() || ref.hasNaN())
        return 1e12;

    double dx = x(0) - ref(0), dy = x(1) - ref(1);
    double pos_cost = std::hypot(dx, dy);
    double dv = x(4) - ref(4);

    double angle_diff = x(2) - ref(2);
    // wrap angle_diff to [-pi, pi]
    angle_diff = std::fmod(angle_diff + 3.141519, 2.0 * 3.141519);
    if (angle_diff < 0)
        angle_diff += 2.0 * 3.141519;
    angle_diff -= 3.141519;

    double cost = pos_cost_weight * pos_cost       // stronger tracking
                  + vel_cost_weight * std::abs(dv) // slow approach
                  + delta_cost_weight * std::abs(angle_diff);

    // terminal stage should be more heavily penalized, unless inside a soft zone
    if (terminal)
    {
        double pos_error = pos_cost; // distance to target
        if (pos_error > terminal_zone_radius)
        {
            cost *= terminal_cost_weight;
        }
    }

    // soft obstacle field, extends eps beyond r_safe
    for (auto &o : obstacles)
    {
        double ox = o(0), oy = o(1), r = o(3);
        double d = std::hypot(x(0) - ox, x(1) - oy);
        if (d < r + obstacle_eps)
        {
            cost += collision_cost_weight * std::pow(r + obstacle_eps - d, 2);
        }
    }
    return cost;
}