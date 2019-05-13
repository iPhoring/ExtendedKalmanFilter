#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::atan2;

/* 
 * Please note that the Eigen library does not initialize 
 * VectorXd or MatrixXd objects with zeros upon creation.
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
/*
* *********Kalman Filter Algorithm**********
   * // Kalman Filter variables
   * VectorXd x;	// object state
   * MatrixXd P;	// object covariance matrix
   * VectorXd u;	// external motion noise
   * MatrixXd F; // state transition matrix
   * MatrixXd H;	// measurement matrix
   * MatrixXd R;	// measurement covariance matrix
   * MatrixXd I; // Identity matrix
   * MatrixXd Q;	// process covariance matrix
   
   * Prediction:
   * x′ = (Fx)+(u)  
   * P′ = [(FP)(F_Transpose)]+Q
   * Measurement Update:
   * y = (z) − (Hx′) //for LASER
   * y = (z) − h(x′) // for RADAR. h is a nonlinear function. No H matrix to map state vector x into polar coordinates. Pls cartesian->polar_coordinates
   * S = [(HP′)(H_transpose)]+R
   * K = (P′)(H_transpose)(S_inverse)
   * New state
   * x = (x′ + Ky)
   * P = (I−KH)(P′)
   * *******************************************
*/
void KalmanFilter::Predict() {
  /**
   * predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   *update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y  = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S  = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K  =  P_ * Ht * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   *update the state by using Extended Kalman Filter equations
   */
  //To calculate y, we use the equations that map the predicted location x' from Cartesian coordinates to polar coordinates
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  //The predicted measurement vector x′ is a vector containing values in the form [px,py,vx,vy]. The radar sensor will output values in polar coordinates.
  double rho = sqrt(px*px + py*py); //range, (ρ), is the distance to the pedestrian
  double phi = atan2(py, px); //φ=atan(py/px). bearing
  double rho_dot = (px*vx + py*vy) / rho; //range rate, ρ_dot, is the projection of the velocity, v, onto direction of object movement

  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;
  VectorXd y = z - h;
  //normalize ϕ subtract 2π from ϕ until it is between -π and π
  while (y(1)>3.1415926){ y(1) -= 2 * 3.1415926;}
  while (y(1)<-3.1415926){y(1) += 2 * 3.1415926;}

  Tools tools;
/* Notes: The Jacobian matrix Hj is used to calculate S,K and P in case of Radar measurement
 * To calculate y, we use the equations that map the predicted location x' from Cartesian coordinates to polar coordinates
*/
  MatrixXd Ht = H_.transpose(); //Pls note that FusionEKF.cpp passes Jacobian matrix Hj in H_
  MatrixXd S  = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K  =  P_ * Ht * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}