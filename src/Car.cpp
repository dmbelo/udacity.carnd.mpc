#include "Car.h"

Car::Car(double a_) { 

  a = a_;
  Reset();

}

Car::~Car() {}

void Car::Simulate(double a_steer, double r_throttle, double dt) {

  x += v * cos(psi) * dt;
  y += v * sin(psi) * dt;
  psi += v / a * a_steer * dt;
  v += r_throttle * dt;
  
}

void Car::Reset() {

  x = 0;
  y = 0;
  psi = 0;
  v = 0;

}