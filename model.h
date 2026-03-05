#ifndef MODEL_H
#define MODEL_H

#include <Eigen/Dense>

class VehicleModel {
public:
    // --- Vehicle physical parameters ---
    double M;   // Vehicle mass (kg)
    double Iz;  // Moment of inertia about Z axis (kg*m^2)
    double a;   // Distance from center of gravity to front axle (m)
    double b;   // Distance from CG to rear axle (m)
    double C;   // Cornering stiffness of the tires
    double mu;  // Friction coefficient
    double g;   // Gravitational acceleration (m/s^2)
    double dt;  // Simulation timestep (s)

    // Constructor
    VehicleModel();

    // Main state prediction function
    void Predict(const Eigen::VectorXd &x, const Eigen::VectorXd &u, Eigen::VectorXd &x_next);

private:
    // Helper physics functions not called externally
    void TireModel(const Eigen::VectorXd &x, Eigen::VectorXd &slip_angles);
    void LateralForce(const Eigen::VectorXd &x, const Eigen::VectorXd &slip_angles, Eigen::VectorXd &lat_force);
};

#endif // MODEL_H