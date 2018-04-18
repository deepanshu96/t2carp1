#include "kalman_filter.h"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

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

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
 VectorXd z_pred, y;
 MatrixXd Ht, S, Si, Pht, K;
 z_pred = H_ * x_;
 y = z - z_pred;
 Ht = H_.transpose();
 S = H_ * P_ * Ht + R_;
 Si = S.inverse();
 Pht = P_ * Ht;
 K = Pht*Si;

 x_ = x_ + (K * y);
 long x_size = x_.size();
 MatrixXd I = MatrixXd::Identity(x_size, x_size);
 P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px,py,vx,vy;
  px = x_(0);
  py = x_(1);
  vx = x_(2);
  vy = x_(3);
  float r,t,rt;

  r = sqrt(px*px + py*py);
  if(px!=0 && py!=0)
    t = atan2(py,px);
  else
    t = atan2(0.0001,0.0001);
  if(fabs(r)<0.0000001)
    {
        px+= 0.0001;
        py+= 0.0001;
        r = sqrt(px*px + py*py);
    }
  rt = (px*vx + py*vy)/(r);

  VectorXd h = VectorXd(3);
  h << r,t,rt;

  VectorXd y = z-h;

  while(y(1) > M_PI){
   y(1) -= M_PI;
 }
 while(y(1) < -M_PI){
   y(1) += M_PI;
 }

  MatrixXd Ht, S, Si, Pht, K;
  Ht = H_.transpose();
  S = H_ * P_ * Ht + R_;
  Si = S.inverse();
  Pht = P_ * Ht;
  K = Pht * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
