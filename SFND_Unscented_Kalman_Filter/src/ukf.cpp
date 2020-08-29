#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.5;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  is_initialized_ = false;

  // state dimension
  n_x_ = 5;

  // augmented state dimension
  n_aug_ = 7;

  // sigma point spread parameter
  lambda_ = 3 - n_x_;

  // sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  // weights
  weights_ = VectorXd(2*n_aug_+1);
  
  // Start time
  time_us_ = 0;

  // NIS for laser
  NIS_laser_ = 0.0;
  
  // NIS for radar
  NIS_radar_ = 0.0;

  step_count_ = 0;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (!is_initialized_)
  {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      double v = sqrt(vx*vx + vy*vy);
      x_ << px, py, v, 0, 0;

      P_ <<  std_radr_*std_radr_, 0, 0, 0, 0,
              0, std_radr_*std_radr_, 0, 0, 0,
              0, 0, std_radrd_*std_radrd_, 0, 0,
              0, 0, 0, std_radphi_*std_radphi_, 0,
              0, 0, 0, 0, 1;
    }
    else
    {
      double px = meas_package.raw_measurements_[0];
      double py = meas_package.raw_measurements_[1];
      x_ << px, py, 0, 0, 0;

      P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
            0, std_laspy_*std_laspy_, 0, 0, 0,
            0, 0, 1, 0, 0, 
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    }

    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    return;
  }
  
  double dt = (meas_package.timestamp_ - time_us_)/1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  // Update step
  if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    UpdateRadar(meas_package);
  }

  if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
    UpdateLidar(meas_package);
  }

  if (radar_count_ > 0 && laser_count_ > 0 && step_count_ % 30 == 0)
  {
    double radar_ratio = (double)radar_over_ / radar_count_;
    double laser_ratio = (double)laser_over_ / laser_count_;
    std::cout<<"radar_ratio = "<<radar_ratio<<"   ";
    std::cout<<"laser_ratio = "<<laser_ratio<<std::endl;
  }

  step_count_ += 1;
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);

  // create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_*std_a_;
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i=0; i<n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
  }

  // predict sigma points
  for (int i = 0; i < 2*n_aug_+1; i++)
  {
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);
    
    // predicted state values
    double px_p, py_p, v_p, yaw_p, yawd_p;
    
    // avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
    } else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }
    
    v_p = v;
    yaw_p = yaw + yawd*delta_t;
    yawd_p = yawd;
    
    // add noise
    px_p += 0.5*delta_t*delta_t*cos(yaw)*nu_a;
    py_p += 0.5*delta_t*delta_t*sin(yaw)*nu_a;
    v_p += delta_t*nu_a;
    yaw_p += 0.5*delta_t*delta_t*nu_yawdd;
    yawd_p += delta_t*nu_yawdd;

    // write predicted sigma point into column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  // set weights
  weights_.fill(0.5/(lambda_+n_aug_));
  weights_(0) = lambda_/(lambda_+n_aug_);

  // predict state mean
  x_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++)
  {
      x_ = x_ + weights_(i)*Xsig_pred_.col(i);
  }

  // predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++)
  {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  int n_z = 2;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // extract radar measurement as z
  VectorXd z = meas_package.raw_measurements_;

  // transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_+1; i++)
  {
      Zsig(0,i) = Xsig_pred_(0,i);
      Zsig(1,i) = Xsig_pred_(1,i);
  }
  
  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++)
  {
      z_pred += weights_(i) * Zsig.col(i);
  }
  
  // calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++)
  {
    // state difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  
  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;
  S = S + R;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;
  while(z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
  while(z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // calculate radar NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  laser_count_ += 1;
  if (NIS_laser_ > 5.99) laser_over_ += 1;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  int n_z = 3;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // extract radar measurement as z
  VectorXd z = meas_package.raw_measurements_;

  // transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_+1; i++)
  {
      // extract values for better readability
      double p_x = Xsig_pred_(0,i);
      double p_y = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);
      double yawd = Xsig_pred_(4,i);
      
      Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);
      Zsig(1,i) = atan2(p_y,p_x);
      Zsig(2,i) = (p_x*cos(yaw)*v + p_y*sin(yaw)*v)/Zsig(0,i);
  }
  
  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++)
  {
      z_pred += weights_(i) * Zsig.col(i);
  }
  
  // calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++)
  {
    // state difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  
  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0,std_radrd_*std_radrd_;
  S = S + R;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;
  while(z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
  while(z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // calculate radar NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  radar_count_ += 1;
  if (NIS_radar_ > 7.81) radar_over_ += 1;
}