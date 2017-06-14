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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  //the initial transition matrix F_
  MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::PolarToCartesian(const MeasurementPackage &measurement_pack, VectorXd &x_) {
    double ro = measurement_pack.raw_measurements_[0];
    double theta = measurement_pack.raw_measurements_[1];
    double px = ro * cos(theta);
    double py = ro * sin(theta);
    x_ << px, py, 0, 0;
}

void FusionEKF::Cartesian(const MeasurementPackage &measurement_pack, VectorXd &x_) {
    x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    ekf_.x_ = VectorXd(4);
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
       PolarToCartesian(measurement_pack, ekf_.x_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
        Cartesian(measurement_pack, ekf_.x_);
    }
    ekf_.Q_ = MatrixXd(4, 4);
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  cout << "Predict Start" << endl;
  ekf_.Predict();
  cout << "Predict End" << endl;

  // Set the process covariance matrix Q
  float ax2 = 9;
  float ay2 = 9;
  float t4ax = pow(dt, 4)/4 * ax2;
  float t3ax = pow(dt, 3)/2 * ax2;
  float t2ax = pow(dt, 2) * ax2;
  float t4ay = pow(dt, 4)/4 * ay2;
  float t3ay = pow(dt, 3)/2 * ay2;
  float t2ay = pow(dt, 2) * ay2;
  ekf_.Q_ << t4ax,    0, t3ax,    0,
           0, t4ay,    0, t3ay,
        t3ax,    0, t2ax,    0,
           0, t3ay,    0, t2ay;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  cout << "Update Start" << endl;
  Eigen::VectorXd x_;
  x_ = VectorXd(4);
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    PolarToCartesian(measurement_pack, x_);
    ekf_.Update(x_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    Cartesian(measurement_pack, x_);
    ekf_.Update(x_);
  }
  cout << "Update End" << endl;

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
