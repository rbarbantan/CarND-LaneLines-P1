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
   * DONE: Calculate the RMSE here.
   */
  VectorXd rmse = VectorXd(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() > 0 && estimations.size() == ground_truth.size()) {
     for (auto i=0U; i < estimations.size(); ++i) {
        VectorXd residuals = estimations[i] - ground_truth[i];
        residuals = residuals.array() * residuals.array();
        rmse += residuals;
     }
     rmse = rmse/estimations.size();
     rmse = rmse.array().sqrt();
  } else {
     std::cout << "Could not compute RMSE, invalid data!" << std::endl;
  }
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * DONE:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj = MatrixXd(3,4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

   // precompute coefficients
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  // check for dvision by zero
  if (c1 < eps_) {
     std::cout << "Division by zero!" << std::endl;
     return Hj;
  }

  // compute Jacobian
  Hj << px/c2,               py/c2,               0,     0,
        -py/c1,              px/c1,               0,     0,
        py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2, py/c2;

   return Hj;
}
