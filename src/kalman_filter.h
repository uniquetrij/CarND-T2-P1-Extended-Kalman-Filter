#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    /**
     * Constructor
     */
    KalmanFilter();

    /**
     * Destructor
     */
    virtual ~KalmanFilter();

    /**
     * Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     */
    void initState(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in);

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param F_in Transition matrix
     * @param Q_ Process covariance matrix
     */
    void predictState(Eigen::MatrixXd F_, Eigen::MatrixXd Q_);

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     * @param H_ Measurement matrix
     * @param R_ Measurement covariance matrix
     */
    void updateState(const Eigen::VectorXd &z, Eigen::MatrixXd H_, Eigen::MatrixXd R_);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     * @param H_ Non-Linear Mapping Function
     * @param R_ Measurement covariance matrix
     */
    void updateState(const Eigen::VectorXd &z, Eigen::VectorXd(*H_)(Eigen::VectorXd), Eigen::MatrixXd R_);

private:
    void estimate(Eigen::MatrixXd& H_, Eigen::MatrixXd& R_, Eigen::VectorXd& y);

};

#endif /* KALMAN_FILTER_H_ */
