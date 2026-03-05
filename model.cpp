#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include "model.h"

// Helper sign function (prevents divide-by-zero issues)
double sign(double val)
{
    if (val > 0.0)
        return 1.0;
    if (val < 0.0)
        return -1.0;
    return 0.0;
}

VehicleModel::VehicleModel()
{
    // Typical family car parameters (e.g. ~1500kg sedan/hatchback)
    M = 1500.0;  // Typical mass
    Iz = 2500.0; // Typical yaw moment of inertia
    a = 1.2;     // Distance to front axle
    b = 1.5;     // Distance to rear axle
    g = 9.81;    // Standard gravity
    dt = 0.01;   // Time step (10 milliseconds)

    // Specific tire/road parameters from the figure
    C = 1200.0; // Cornering stiffness
    mu = 0.85;  // Typical friction coefficient for dry asphalt
}

void VehicleModel::Predict(const Eigen::VectorXd &x, const Eigen::VectorXd &u, Eigen::VectorXd &x_next)
{
    // if input state is invalid, zero output and return
    for (int i = 0; i < x.size(); ++i) {
        if (std::isnan(x(i)) || std::isinf(x(i))) {
            x_next = Eigen::VectorXd::Zero(x.size());
            return;
        }
    }
    // First compute tire forces based on current state x
    Eigen::VectorXd slip_angles(2);
    Eigen::VectorXd lat_force(2);

    TireModel(x, slip_angles);
    LateralForce(x, slip_angles, lat_force);

    double FyF = lat_force(0);
    double FyR = lat_force(1);

    // Read state variables (for readability)
    double psi = x(2);
    double beta = x(3);
    double vx = x(4);
    double vy = x(5);
    double r = x(6);
    double delta = x(7);
    double Fx = x(8);

    // Inputs
    double delta_des = u(0);
    double Fx_des = u(1);

    // small safeguard against divide-by-zero when vx is near zero
    if (std::abs(vx) < 0.01)
    {
        vx = 0.01 * sign(vx + 1e-5); // Safeguard
    }

    // --- DERIVATIVES CALCULATIONS (formulas from figure) ---
    double x_dot = vx * std::cos(psi) - vy * std::sin(psi);
    double y_dot = vx * std::sin(psi) + vy * std::cos(psi);
    double psi_dot = r;
    double beta_dot = (FyF + FyR) / (M * vx) - r;
    double vx_dot = (Fx - FyF * std::sin(delta)) / M + r * vx * beta;
    double vy_dot = (FyF + FyR) / M - r * vx;
    double r_dot = (a * FyF - b * FyR) / Iz;

    // NOTE: paper writes 10*(delta - delta_des) but that gives positive
    // feedback and destabilizes the system. correct is 10*(delta_des - delta).
    double delta_dot = 10.0 * (delta_des - delta);
    double Fx_dot = 10.0 * (Fx_des - Fx);

    // --- EULER INTEGRATION (x_next = x + x_dot * dt) ---
    x_next(0) = x(0) + x_dot * dt;
    x_next(1) = x(1) + y_dot * dt;
    x_next(2) = x(2) + psi_dot * dt;
    x_next(3) = x(3) + beta_dot * dt;
    x_next(4) = x(4) + vx_dot * dt;
    x_next(5) = x(5) + vy_dot * dt;
    x_next(6) = x(6) + r_dot * dt;
    x_next(7) = x(7) + delta_dot * dt;
    x_next(8) = x(8) + Fx_dot * dt;
}

void VehicleModel::TireModel(const Eigen::VectorXd &x, Eigen::VectorXd &slip_angles)
{
    double beta = x(3);
    double vx = x(4);
    double r = x(6);
    double delta = x(7);

    if (std::abs(vx) < 0.01)
        vx = 0.01 * sign(vx + 1e-5); // safeguard

    // Formulas for alpha_F and alpha_R from the figure (use beta, not v_y!)
    double slip_angle_front = std::atan(beta + a * r / vx) - delta;
    double slip_angle_rear = std::atan(beta - b * r / vx);

    slip_angles << slip_angle_front, slip_angle_rear;
}

void VehicleModel::LateralForce(const Eigen::VectorXd &x, const Eigen::VectorXd &slip_angles, Eigen::VectorXd &lat_force)
{
    // Normal force (I simply divide the weight by 4; in reality it varies with a and b)
    double Fz = M * g / 4.0;

    for (int i = 0; i < 2; ++i)
    {
        double alpha = slip_angles(i);

        // IMPORTANT: The text says "Fx is the longitudinal force imparted by the rear wheels".
        // So Fx = 0 for front wheel (i==0), and Fx = x(8) for rear wheel (i==1).
        double Fx_i = (i == 1) ? x(8) : 0.0;

        // Guard against negative under the sqrt (mu^2 * Fz^2 - Fx^2)
        double inner_sqrt = std::pow(mu * Fz, 2) - std::pow(Fx_i, 2);
        if (inner_sqrt < 0)
            inner_sqrt = 0.0;

        // 1. First compute xi (zeta)!
        double xi = std::sqrt(inner_sqrt) / (mu * Fz);

        // If alpha is zero the expressions are undefined; skip to next iteration.
        if (std::abs(alpha) < 1e-6)
        {
            lat_force(i) = 0.0;
            continue;
        }

        // 2. Next compute gamma. (abs(alpha)/alpha is basically sign(alpha))
        double gamma = std::abs(std::atan(3.0 * xi * Fz * sign(alpha)));

        // Brush Tire Model
        if (std::abs(alpha) >= gamma)
        {
            lat_force(i) = -mu * xi * Fz * sign(alpha);
        }
        else
        {
            double tan_a = std::tan(alpha);

            double term1 = -C * tan_a;
            double term2 = (std::pow(C, 2) / (3.0 * xi * mu * Fz)) * (std::pow(tan_a, 3) / std::abs(tan_a));
            double term3 = -(std::pow(C, 3) / (27.0 * std::pow(mu, 2) * std::pow(xi, 2) * std::pow(Fz, 2))) * std::pow(tan_a, 3);

            lat_force(i) = term1 + term2 + term3;
        }
    }
}