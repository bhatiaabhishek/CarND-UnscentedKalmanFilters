#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1  ;

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
  
  n_aug_ = 7;
  n_x_ = 5;

  lambda_ = 3 - n_aug_;
  previous_timestamp_ = 0;
 
  P_ << 0.85, 0, 0, 0, 0,
        0 , 0.85, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 0.5, 0,
        0, 0, 0, 0, 0.5;
  
  //create vector for weights
  weights_ = VectorXd(2*n_aug_+1);
  
  //set weights
  double weights_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weights_0;
  double wi = 0.5/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1;i++) {
      weights_(i) = wi;
  }
        
       
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    
    if (!is_initialized_) {

        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
          /**
          Convert radar from polar to cartesian coordinates and initialize state.
          */
          double ro = meas_package.raw_measurements_(0);
          double phi = meas_package.raw_measurements_(1);
          double ro_v = meas_package.raw_measurements_(2);
          x_ << ro*cos(phi), ro*sin(phi), 0,0, 0;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
          /**
          Initialize state.
          */
          x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 6, -0.9, 0;
        }

     if (fabs(x_(0)) < 0.001 and fabs(x_(1)) < 0.001){
                x_(0) = 0.001;
                x_(1) = 0.001;
        }
    // done initializing, no need to predict or update
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

    double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;   //dt - expressed in seconds
    previous_timestamp_ = meas_package.timestamp_;




    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

        //std::cout << "delta t : " << std::endl;
        //std::cout << dt << std::endl;
        // Prediction
        Prediction(dt);
        //std::cout << "x pred : " << std::endl;
        //std::cout << x_ << std::endl;
        //std::cout << "P pred : " << std::endl;
        //std::cout << P_ << std::endl;
        if (isnan(x_(0)) == 1){ throw std::exception(); }
        bool isRadar = true;
        VectorXd z_out = VectorXd(3);
        MatrixXd S_out = MatrixXd(3, 3);
        //create matrix for cross correlation Tc
        MatrixXd Tc = MatrixXd(n_x_, 3);
        VectorXd Z = VectorXd(3);
        Z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1), meas_package.raw_measurements_(2); 
        MatrixXd R = MatrixXd(3,3);
        R << pow(std_radr_,2), 0 , 0,
             0 , pow(std_radphi_,2), 0,
             0, 0, pow(std_radrd_,2);
        PredictMeasurement(isRadar, R, 3, &z_out,&S_out,&Tc);
        UpdateState(isRadar, Z, z_out, S_out, Tc, 3, &x_, &P_,&NIS_radar_);

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

        Prediction(dt);
        if (isnan(x_(0)) == 1){ throw std::exception(); }
        bool isRadar = false;
        MatrixXd R = MatrixXd(2,2);
        R << pow(std_laspx_,2), 0 ,
             0 , pow(std_laspy_,2);
        VectorXd z_out = VectorXd(2);
        MatrixXd S_out = MatrixXd(2, 2);
        //create matrix for cross correlation Tc
        MatrixXd Tc = MatrixXd(n_x_, 2);
        VectorXd Z = VectorXd(2);
        Z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1); 
        PredictMeasurement(isRadar, R, 2, &z_out,&S_out,&Tc);
        UpdateState(isRadar, Z, z_out, S_out, Tc, 2, &x_, &P_,&NIS_laser_);
        


    }
}





/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(n_aug_-2) = 0;
  x_aug(n_aug_-1) = 0;
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_aug_-2,n_aug_-2) = std_a_*std_a_;
  P_aug(n_aug_-1,n_aug_-1) = std_yawdd_*std_yawdd_;
  //std::cout << "P aug pred : " << std::endl;
  //std::cout << P_aug << std::endl;
  
 //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
 //std::cout << " A : " << std::endl;
 //std::cout << A << std::endl;
 //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
 
 for (int i=0;i<n_aug_;i++) {
     Xsig_aug.col(i+1) = x_aug + (sqrt(lambda_+n_aug_)*A.col(i));
     Xsig_aug.col(i+1+n_aug_) = x_aug - (sqrt(lambda_+n_aug_)*A.col(i));
 }


 // Sigma point prediction
 Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

 VectorXd T1 = VectorXd(n_x_,1);
 VectorXd Q = VectorXd(n_x_,1);
 VectorXd X_k = VectorXd(n_x_,1);
 for (int i=0;i < 2*n_aug_ +1; i++) {
     VectorXd T1 = VectorXd(n_x_,1);
     VectorXd Q = VectorXd(n_x_,1);
     VectorXd X_k = VectorXd(n_x_,1);
     double v_x = Xsig_aug(2,i);
     double W_x = Xsig_aug(3,i);
     double W_x_dot = Xsig_aug(4,i);
     double V_ak = Xsig_aug(5,i);
     double V_yk = Xsig_aug(6,i);
     if (fabs(W_x_dot) > 0.001) {
     T1 <<  (v_x/W_x_dot) * (sin(W_x + (W_x_dot*delta_t)) - sin(W_x)), 
            (v_x/W_x_dot) * (-cos(W_x + (W_x_dot*delta_t)) + cos(W_x)),
            0,
            W_x_dot*delta_t,
            0;
     }
     else {
         T1 <<  v_x*delta_t*cos(W_x), 
                v_x*delta_t*sin(W_x),
                0,
                 W_x_dot*delta_t,
                0;
     }
     //std::cout << "I am here" << std::endl;
     Q << 0.5*pow(delta_t,2)*V_ak*cos(W_x),
          0.5*pow(delta_t,2)*V_ak*sin(W_x),
          delta_t*V_ak,
          0.5*pow(delta_t,2)*V_yk,
          delta_t*V_yk;
     for (int j =0 ; j< n_x_; j++) {
         X_k(j) = Xsig_aug(j,i);
         
     }   
     Xsig_pred_.col(i) = X_k + T1 + Q;
   }

  // ##################
  //Prediction of mean of covariance from Xsig_pred
 
  
  //predict state mean
  x_.fill(0.0);
  
  for (int j=0;j<2*n_aug_+1;j++) {
      x_ = x_ + (weights_(j)*Xsig_pred_.col(j));
      
  }
 
  //predict state covariance matrix
  P_.fill(0.0);
  VectorXd X1 = VectorXd(n_x_);
  MatrixXd X1_t = MatrixXd(1,n_x_);
  for (int j=0;j<2*n_aug_+1;j++) {
      X1 = Xsig_pred_.col(j)-x_;
      X1_t = X1.transpose();
      P_ = P_ + (weights_(j)*(X1*X1_t));
  
   }


 }  

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {



}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {


}


void UKF::PredictMeasurement(bool isRadar,MatrixXd R, int n_z, VectorXd* z_out, MatrixXd* S_out, MatrixXd* Tc_out) {


  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

   
  //transform sigma points into measurement space
  for (int i=0;i<2*n_aug_+1;i++) {
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double phi = Xsig_pred_(3,i);
      if (fabs(px) < 0.001) {
         if (px > 0) { px = 0.001;}
         else { px = -0.001; }
       }
      if (fabs(py) < 0.001) {
         if (py > 0) { py = 0.001;}
         else { py = -0.001; }
       }
      if (isRadar) { 
          Zsig(0,i) = sqrt(pow(px,2) + pow(py,2));
          Zsig(1,i) = atan2(py,px);
          Zsig(2,i) = ((px*v*cos(phi)) + (py*v*sin(phi)))/ Zsig(0,i);
      }
      else { // LIDAR measurement model
          Zsig(0,i) = px;
          Zsig(1,i) = py;
      } 
  }
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int j=0;j<2*n_aug_+1;j++) {
      z_pred = z_pred + (weights_(j)*Zsig.col(j));
  }
  if (isRadar) {
    while (z_pred(1) > M_PI) { z_pred(1) -= 2*M_PI; }
    while (z_pred(1) < -M_PI) { z_pred(1) += 2*M_PI; }
  }
  //calculate measurement covariance matrix S

  S.fill(0.0);
  VectorXd X1 = VectorXd(n_z);
  MatrixXd X1_t = MatrixXd(1,n_z);
  for (int j=0;j<2*n_aug_+1;j++) {
      X1 = Zsig.col(j)-z_pred;
      if (isRadar) {
        while (X1(1) > M_PI) { X1(1) -= 2*M_PI; }
        while (X1(1) < -M_PI) { X1(1) += 2*M_PI; }
      }
      X1_t = X1.transpose();
      S = S + (weights_(j)*(X1*X1_t));
  
   }
        
   S = S + R;

  //std::cout << " S : " << std::endl;
  //std::cout << S << std::endl;

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  VectorXd Z1 = VectorXd(n_z);
  MatrixXd Z1_t = MatrixXd(1,n_z);
  for (int i=0; i<2*n_aug_+1;i++) {
      
    Z1 = (Zsig.col(i) - z_pred);
    if (isRadar) {
      while (Z1(1) > M_PI) { Z1(1) -= 2*M_PI; }
      while (Z1(1) < -M_PI) { Z1(1) += 2*M_PI; }
    }
    Z1_t = Z1.transpose();
    Tc = Tc + (weights_(i)*(Xsig_pred_.col(i) - x_)*Z1_t);
  }

    //std::cout << " Tc : " << std::endl;
    //std::cout << Tc << std::endl;
    //std::cout << " z_pred : " << std::endl;
    //std::cout << z_pred << std::endl;
  *z_out = z_pred;
  *S_out = S;
  *Tc_out = Tc;

}


void UKF::UpdateState(bool isRadar, VectorXd z, VectorXd z_pred, MatrixXd S, MatrixXd Tc, int n_z, VectorXd* x_out, MatrixXd* P_out, double* NIS_out) {

  VectorXd z_error = VectorXd(n_z);
  MatrixXd z_error_T = MatrixXd(1,n_z);
  //calculate Kalman gain K;
  MatrixXd S_inv = MatrixXd(n_z,n_z);
  S_inv = S.inverse();
  MatrixXd K = MatrixXd(n_x_, n_z);
  K = Tc*S_inv;
  //update state mean and covariance matrix
  z_error = z-z_pred;
  if (isRadar) {
  while (z_error(1) > M_PI) { z_error(1) -= 2*M_PI; }
  while (z_error(1) < -M_PI) { z_error(1) += 2*M_PI; }
  }
  *x_out = *x_out + (K*z_error);

  MatrixXd K_t = MatrixXd(n_z,n_x_);
  K_t = K.transpose();
  
  *P_out = *P_out - (K*S*K_t);

  z_error_T = z_error.transpose();
  MatrixXd NIS_temp  = z_error_T*S_inv*z_error;
  *NIS_out = NIS_temp(0);
  


}
