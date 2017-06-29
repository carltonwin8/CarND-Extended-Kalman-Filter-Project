#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    if (estimations.size() == 0) return rmse;
    if (estimations.size() != ground_truth.size()) return rmse;

    for(int i=0; i < estimations.size(); ++i){
        VectorXd diff = ground_truth[i] - estimations[i];
        VectorXd diff2 = diff.array() * diff.array();
        rmse += diff2;
    }

    rmse /= estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
    Hj = MatrixXd::Zero(3,4);

    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float px2 = px * px;
    float py2 = py * py;
    float px2py2 = px2 + py2;
    if (((px == 0) && (py == 0)) || (px2py2 < 0.01)) {
        cout << "CalculateJacotian () - Error" << endl;
        return Hj;
    } else {
        float px2py2sqrt = pow(px2py2, 0.5);
        float px2py2_3d2 = px2py2 * px2py2sqrt;
        Hj << px/px2py2sqrt, py/px2py2sqrt, 0, 0,
             -py/px2py2, px/px2py2, 0, 0,
             py*(vx*py - vy*px)/px2py2_3d2, px*(vy*px - vx*py)/px2py2_3d2, px/px2py2sqrt, py/px2py2sqrt;
    }
    return Hj;
}
