#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
    /**
     * Constructor.
     */
    FusionEKF();

    /**
     * Destructor.
     */
    virtual ~FusionEKF();

    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /**
     * Non-linear mapping function for radar
     */
    static Eigen::VectorXd h_Radar(const Eigen::VectorXd x_);

    /**
     * Inverse of the Non-linear mapping function for radar
     */
    static Eigen::VectorXd h_inv_Radar(const Eigen::VectorXd x_);

    /**
     * Kalman Filter update and prediction math lives in here.
     */
    KalmanFilter ekf_;

private:
    // check whether the tracking toolbox was initialized or not (first measurement)
    bool is_initialized_;

    // previous timestamp
    long long previous_timestamp_;

    // tool object used to compute Jacobian and RMSE
    Tools tools;

    Eigen::VectorXd X_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;
    Eigen::VectorXd(*H_radar)(Eigen::VectorXd);
    Eigen::MatrixXd F_;
    Eigen::MatrixXd F_radar_;
    Eigen::MatrixXd Q_;

    //acceleration noise components
    float noise_ax;
    float noise_ay;
};

#endif /* FusionEKF_H_ */
