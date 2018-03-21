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
    X_ = VectorXd(4);
    P_ = MatrixXd(4, 4);
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    H_radar = &h_Radar;
    F_ = MatrixXd(4, 4);

    P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    R_laser_ << 0.0225, 0,
            0, 0.0225;

    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    H_radar = &h_Radar;

    F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Q_ = MatrixXd(4, 4);

    noise_ax = 9;
    noise_ay = 9;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {
}

Eigen::VectorXd FusionEKF::h_Radar(const Eigen::VectorXd x_) {
    float px = x_(0); // position in x
    float py = x_(1); // position in y
    float vx = x_(2); // velocity in x
    float vy = x_(3); // velocity in y
    // Coordinates conversion from cartesian to polar
    float rho = sqrt(pow(px, 2) + pow(py, 2));
    if (rho < 0.0001) {
        px += 0.0001;
        py += 0.0001;
        rho = sqrt(px * px + py * py);
    }
    float phi = atan2(py, px);
    float dot = (px * vx + py * vy) / rho;

    VectorXd h(3);
    h << rho, phi, dot;
    return h;
}

Eigen::VectorXd FusionEKF::h_inv_Radar(const Eigen::VectorXd x_) {
    double rho = x_[0]; // range
    double phi = x_[1]; // bearing
    double dot = x_[2]; // rho dot
    // Coordinates conversion from polar to cartesian
    double x = rho * cos(phi);
    if (x < 0.0001) {
        x = 0.0001;
    }
    double y = rho * sin(phi);
    if (y < 0.0001) {
        y = 0.0001;
    }
    double vx = dot * cos(phi);
    double vy = dot * sin(phi);

    VectorXd h_inv(4);

    h_inv << x, y, vx, vy;
    return h_inv;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            X_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2], 0;
            X_ = h_inv_Radar(X_);
            ekf_.initState(X_, P_);
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            X_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
            ekf_.initState(X_, P_);
        }

        // done initializing, no need to predict or update
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    //Modify the F matrix so that the time is integrated
    F_(0, 2) = dt;
    F_(1, 3) = dt;

    Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
            0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
            dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
            0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;


    ekf_.predictState(F_, Q_);

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.updateState(measurement_pack.raw_measurements_, H_radar, R_radar_);
    } else {
        ekf_.updateState(measurement_pack.raw_measurements_, H_laser_, R_laser_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
