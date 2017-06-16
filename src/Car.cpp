#include "Car.h"

Car::Car(double a_) { 

  a = a_;
  Reset();

}

Car::~Car() {}

void Car::Step(double a_steer, double g_accel, double dt) {

  x += v * cos(psi) * dt;
  y += v * sin(psi) * dt;
  psi += v / a * a_steer * dt;
  v += g_accel * dt;
  cte += v * sin(err_psi) * dt;
  err_psi += v / a * a_steer * dt;
  
}

void Car::Set(double x_, double y_, double psi_, double v_, double cte_, double err_psi_) {

  x = x_;
  y = y_;
  psi = psi_;
  v = v_;
  cte = cte_;
  err_psi = err_psi_;

}

void Car::Reset() {

  x = 0;
  y = 0;
  psi = 0;
  v = 0;
  cte = 0;
  err_psi = 0;

}