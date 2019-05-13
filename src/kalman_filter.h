#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial object state
   * @param P_in Initial object state covariance matrix
   * @param F_in State Transition matrix
   * 
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   * 
   * 
   * *********Kalman Filter Algorithm**********
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
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();
  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
};

#endif // KALMAN_FILTER_H_
