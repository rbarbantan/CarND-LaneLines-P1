#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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
   * DONE: Finish initializing the FusionEKF.
   * DONE: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * DONE: Initialize the state ekf_.x_ with the first measurement.
     * DONE: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement, we mostly care about initializing the state 
    // and setting the other matrices with some default values.
    cout << "EKF: " << endl;
    VectorXd init_state = VectorXd(4);
    MatrixXd P = MatrixXd(4, 4);
    MatrixXd F = MatrixXd(4, 4);
    MatrixXd Q = MatrixXd(4, 4);
    MatrixXd H;
    MatrixXd R;

    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1000, 0,
         0, 0, 0, 1000;
    
    F << 1, 0, 1, 0,
         0, 1, 0, 1,
         0, 0, 1, 0,
         0, 0, 0, 1;

    Q << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // DONE: Convert radar from polar to cartesian coordinates and initialize state.
      float range = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      float range_rate = measurement_pack.raw_measurements_[2];

      init_state(0) = range * cos(theta);
      init_state(1) = range * sin(theta);
      init_state(2) = range_rate * cos(theta);
      init_state(3) = range_rate * sin(theta);
  
      H = tools.CalculateJacobian(init_state);
      R = R_radar_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // DONE: Initialize state.
      float x = measurement_pack.raw_measurements_[0];
      float y = measurement_pack.raw_measurements_[1];
      init_state << x, y, 0 , 0;
      
      H = H_laser_;
      R = R_laser_;
    }

    ekf_.Init(init_state, P, F, H, R, Q);

    previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * DONE: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  /**
   * DONE: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float noise_ax = 9;
  float noise_ay = 9;
  
  float dt_2 = dt * dt;
  float dt_3 = dt * dt_2;
  float dt_4 = dt * dt_3;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * DONE:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // DONE: Radar updates
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // DONE: Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
