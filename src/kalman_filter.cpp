#include <iostream>
#define PI 3.14159265

#include "kalman_filter.h"

#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::initState(VectorXd &x_in, MatrixXd &P_in) {
    x_ = x_in;
    P_ = P_in;
}

void KalmanFilter::predictState(MatrixXd F_, MatrixXd Q_) {
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::estimate(MatrixXd& H_, MatrixXd& R_, VectorXd& y) {
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::updateState(const VectorXd &z, MatrixXd H_, MatrixXd R_) {

    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    estimate(H_, R_, y);
}

void KalmanFilter::updateState(const Eigen::VectorXd &z, Eigen::VectorXd(*h)(Eigen::VectorXd), MatrixXd R_) {

    MatrixXd H_ = Tools::CalculateJacobian(x_);
    VectorXd z_pred = h(x_);
    VectorXd y = z - z_pred;

    if (y[1] > PI)
        y[1] -= 2.f * PI;
    if (y[1] < -PI)
        y[1] += 2.f * PI;

    estimate(H_, R_, y);
}
