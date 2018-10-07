#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

#define M_TAU (2 * M_PI)

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}

double normalize_angle(double angle) {

    while (angle > +M_PI) angle -= M_TAU;

    while (angle < -M_PI) angle += M_TAU;

    return angle;
}

VectorXd cartesian_to_polar(const VectorXd &cartesian) {

    // extract cartesian components
    double px = cartesian(0);
    double py = cartesian(1);
    double vx = cartesian(2);
    double vy = cartesian(3);

    // compute polar components
    double rho = sqrt(px*px + py*py);
    double phi = atan2(py, px);
    double rho_dot = (px * vx + py * vy) / rho;

    // create and return polar vector
    VectorXd polar = VectorXd(3);
    polar << rho, phi, rho_dot;

    return polar;
}

VectorXd polar_to_cartesian(const VectorXd &polar) {

    // extract polar components
    double rho     = polar(0);
    double phi     = polar(1);
    double rho_dot = polar(2);

    // compute cartesian components
    double  x = cos(phi) * rho;
    double  y = sin(phi) * rho;
    double vx = cos(phi) * rho_dot;
    double vy = sin(phi) * rho_dot;

    // create and return cartesian vector
    VectorXd cartesian = VectorXd(4);
    cartesian << x, y, vx, vy;

    return cartesian;
}
