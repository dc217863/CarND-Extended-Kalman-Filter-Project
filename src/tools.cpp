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

    assert(estimations.size() != 0 && estimations.size() == ground_truth.size());

    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd residual = (estimations[i] - ground_truth[i]);
        residual = residual.array()*residual.array(); //coefficient-wise multiplication
        rmse += residual;
    }

    //calculate the mean
    // ... your code here
    rmse = rmse/estimations.size();

    //calculate the squared root
    // ... your code here
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float px_2_plus_py_2 = px*px + py*py;
	float sqrt_px_2_plus_py_2 = sqrt(px_2_plus_py_2);

	Hj << (px/sqrt_px_2_plus_py_2), (py/sqrt_px_2_plus_py_2), 0, 0,
	      (-py/px_2_plus_py_2), (px/px_2_plus_py_2), 0, 0,
	      (py*(vx*py - vy*px)/pow(px_2_plus_py_2, 1.5)), (px*(vy*px - vx*py)/pow(px_2_plus_py_2, 1.5)), px/sqrt_px_2_plus_py_2, py/sqrt_px_2_plus_py_2;
	return Hj;
}
