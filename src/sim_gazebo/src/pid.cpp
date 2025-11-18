#include "pid.hpp"

PID::PID(double dp, double di, double dd) : dp_(dp), di_(di), dd_(dd), integral_(0.0), previous_error_(0.0) {}

double PID::compute(double setpoint, double actual, double dt) {
   double error = setpoint - actual;
   integral_ += error * dt;
   double derivative = (error - previous_error_) / dt;
   double out = dp_ * error + di_ * integral_ + dd_ * derivative;
   previous_error_ = error;
   return out;
}