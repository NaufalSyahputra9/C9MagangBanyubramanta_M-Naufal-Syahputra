#ifndef PID_HPP
#define PID_HPP

class PID {
 public:
   PID(double dp, double di, double dd);
   double compute(double setpoint, double actual, double dt);

 private:
   double dp_;
   double di_;
   double dd_;
   double integral_;
   double previous_error_;
};

#endif 