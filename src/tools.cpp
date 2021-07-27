#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd l_rmse_v(4);
   l_rmse_v << 0, 0, 0, 0;
   
   // Getting the squared difference
   for (unsigned int i=0;i < estimations.size(); ++i){
      VectorXd l_residual_v = estimations[i] - ground_truth[i];
      l_residual_v = l_residual_v.array()*l_residual_v.array();
      l_rmse_v += l_residual_v;
   }
   
   // taking the average value for rmse
   l_rmse_v /= estimations.size();
   l_rmse_v = l_rmse_v.array().sqrt();
   return l_rmse_v;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd l_Hj_m(3, 4);
  // state variables
  float l_px_f = x_state(0);
  float l_py_f = x_state(1);
  float l_vx_f = x_state(2);
  float l_vy_f = x_state(3);

  float l_c1_f = l_px_f*l_px_f + l_py_f*l_py_f;    // l_c1_f = l_px_f^2 + l_py_f^2
  float l_c2_f = sqrt(l_c1_f);                     // l_c2_f = sqrt(l_px_f^2 + l_py_f^2)
  float l_c3_f = (l_c1_f*l_c2_f);                  // l_c3_f = (l_px_f^2 + l_py_f^2)^(3/2)

  if (fabs(l_c1_f) < 0.0001){
    std::cout << "Division by Zero!" << std::endl;
    return l_Hj_m;
  }
  l_Hj_m << (l_px_f/l_c2_f), (l_py_f/l_c2_f), 0, 0,
        -(l_py_f/l_c1_f), (l_px_f/l_c1_f), 0, 0,
        l_py_f*(l_vx_f*l_py_f - l_vy_f*l_px_f) / l_c3_f, l_px_f*(l_px_f*l_vy_f - l_py_f*l_vx_f) / l_c3_f, l_px_f / l_c2_f, l_py_f / l_c2_f;
  
  return l_Hj_m;  
}
