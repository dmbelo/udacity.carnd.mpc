#include "Poly.h"

Poly::Poly() {};
Poly::~Poly() {};

void Poly::Fit(Eigen::VectorXd x_vals, Eigen::VectorXd y_vals, int order) {
    // Fit a polynomial
    // Adapted from:
    // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716                        

    assert(x_vals.size() == y_vals.size());
    assert(order >= 1 && order <= x_vals.size() - 1);
    Eigen::MatrixXd A(x_vals.size(), order + 1);

    for (int i = 0; i < x_vals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < x_vals.size(); j++) {
        for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * x_vals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(y_vals);
    
    coeffs = result;

}