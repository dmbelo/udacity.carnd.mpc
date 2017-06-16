#ifndef CAR_H
#define CAR_H

#include <math.h>

class Car {

  public:
    double a, x, y, psi, v, cte, err_psi;
    Car(double a_);
    virtual ~Car();
    void Step(double a_steer, double g_accel, double dt);
    void Set(double x, double y, double psi, double v, double cte, double err_psi);
    void Reset();
  
};
#endif /* CAR_H */