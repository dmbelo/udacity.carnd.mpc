#ifndef CAR_H
#define CAR_H

#include <math.h>

class Car {

  public:
    double a, x, y, psi, v;
    Car(double a_);
    virtual ~Car();
    void Simulate(double a_steer, double r_throttle, double dt);
    void Reset();
  
};
#endif /* CAR_H */