#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    // measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // laser measurement matrix
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // radar Jacobian matrix
    Hj_ <<  1, 1, 0, 0,
            1, 1, 0, 0,
            1, 1, 1, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {

        // first measurement
        cout << "EKF: " << endl;

        ekf_.P_ = MatrixXd(4, 4);
        ekf_.P_  << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1000, 0,
                    0, 0, 0, 1000;

        ekf_.F_ = MatrixXd(4, 4);
        ekf_.F_  << 1, 0, 1, 0,
                    0, 1, 0, 1,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // convert radar from polar to cartesian coordinates and initialize state.
            ekf_.x_ = polar_to_cartesian(measurement_pack.raw_measurements_);
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            double px = measurement_pack.raw_measurements_(0);
            double py = measurement_pack.raw_measurements_(1);

            // initialize state.
            ekf_.x_ = VectorXd(4);
            ekf_.x_ << px, py, 0, 0;
        }

        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    // compute the time difference, measured in seconds
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;

    // update the timestamp for the next iteration
    previous_timestamp_ = measurement_pack.timestamp_;

    // update the state transition matrix F according to the new elapsed time
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_  << 1,  0, dt,  0,
                0,  1,  0, dt,
                0,  0,  1,  0,
                0,  0,  0,  1;

    // use noise_ax = 9 and noise_ay = 9 for your Q matrix
    double noise_ax = 9;
    double noise_ay = 9;

    double dt_2 = pow(dt, 2);
    double dt_3 = pow(dt, 3);
    double dt_4 = pow(dt, 4);

    // update the process noise covariance matrix (Q)
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_  << (dt_4/4)*noise_ax, 0, (dt_3/2)*noise_ax, 0,
                0, (dt_4/4)*noise_ay, 0, (dt_3/2)*noise_ay,
                (dt_3/2)*noise_ax, 0, (dt_2/1)*noise_ax, 0,
                0, (dt_3/2)*noise_ay, 0, (dt_2/1)*noise_ay;

    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
    } else {
        // Laser updates
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
