#include "kalman_filter.h"

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_*x_;
  MatrixXd l_Ft_m = F_.transpose();
  P_ = F_*P_*l_Ft_m + Q_; 
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd l_zp_v = H_ * x_;
  VectorXd l_y_v = z - l_zp_v;
  MatrixXd l_Ht_m = H_.transpose();
  MatrixXd l_S_m = H_ * P_ * l_Ht_m + R_;
  MatrixXd l_Si_m = l_S_m.inverse();
  MatrixXd l_PHt_m = P_ * l_Ht_m;
  MatrixXd l_K_m = l_PHt_m * l_Si_m;

  // New estimtes
  x_ = x_ + (l_K_m * l_y_v);
  long l_statesize_l = x_.size();
  MatrixXd l_I_m = MatrixXd::Identity(l_statesize_l, l_statesize_l);
  P_ = (l_I_m - (l_K_m * H_)) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  float l_rho_f = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float l_phi_f = atan2(x_(1), x_(0));
  float l_rhodot_f = (fabs(l_rho_f) < 0.0001) ? (0): ((x_(0)*x_(2) + x_(1)*x_(3))/l_rho_f);
  VectorXd l_zp_v(3);
  l_zp_v << l_rho_f, l_phi_f, l_rhodot_f;
  VectorXd l_y_v = z - l_zp_v;

  if( l_y_v[1] > PI )
    l_y_v[1] -= 2.f*PI;
  if( l_y_v[1] < -PI )
    l_y_v[1] += 2.f*PI;

  Tools l_tools;
  MatrixXd l_Hj_m = l_tools.CalculateJacobian( x_ );
  MatrixXd l_Hjt_m = l_Hj_m.transpose();
  MatrixXd l_S_m = l_Hj_m * P_ * l_Hjt_m + R_;
  MatrixXd l_Si_m = l_S_m.inverse();
  MatrixXd l_K_m = P_ * l_Hjt_m * l_Si_m;

  // New estimtes
  x_ = x_ + (l_K_m * l_y_v);
  long state_size = x_.size();
  MatrixXd I = MatrixXd::Identity(state_size, state_size);
  P_ = (I - (l_K_m * H_)) * P_;  
}
