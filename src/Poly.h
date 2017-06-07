#ifndef POLY_H
#define POLY_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

class Poly {
    public:
        Eigen::VectorXd coeffs;
        Poly();
        virtual ~Poly();
        void Fit(Eigen::VectorXd x_vals, Eigen::VectorXd y_vals, int order);
        double Eval(double x);
        double Diff(double x);
};

#endif /* POLY_H */