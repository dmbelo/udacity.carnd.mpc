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
        
        template <class Type>
        Type Eval(const Type x) {
            Type result = 0.0;
            for (int i = 0; i < coeffs.size(); i++) {
                result += coeffs[i] * pow(x, i);
            }
    
            return result;

        }
        
        template <class Type>
        Type Diff(const Type x) {
            Type result = 0.0;
            for (unsigned int i = 1; i < coeffs.size(); i++) {
                result += i * coeffs[i] * pow(x, i - 1);
            }

            return result;
        }
};

#endif /* POLY_H */