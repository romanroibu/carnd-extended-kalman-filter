#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
    /**
     * Constructor.
     */
    Tools();

    /**
     * Destructor.
     */
    virtual ~Tools();

    /**
     * A helper method to calculate RMSE.
     */
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

    /**
     * A helper method to calculate Jacobians.
     */
    MatrixXd CalculateJacobian(const VectorXd& x_state);

};

/// Normalize the angle between `-pi` to `pi`.
double normalize_angle(double angle);

/// Convert cartesian measurements (x, y, vx, vy) to polar measurements (rho, phi, rho_dot).
VectorXd cartesian_to_polar(const VectorXd&);

/// Convert polar measurements (rho, phi, rho_dot) to cartesian measurements (x, y, vx, vy).
VectorXd polar_to_cartesian(const VectorXd&);

#endif /* TOOLS_H_ */
