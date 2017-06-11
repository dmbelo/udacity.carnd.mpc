#include <math.h>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Poly.h"
#include "matplotlibcpp.h"

#define _USE_MATH_DEFINES
#define VERY_LARGE_NUMBER 1.0e15

using CppAD::AD;
using std::vector;
namespace plt = matplotlibcpp;

double t_horizon = 1;

size_t n_states = 6;
size_t n_steps = 3;
size_t n_actuators = 2;
size_t n_constraints = n_steps * n_states; 
size_t n_vars = n_states * n_steps + n_actuators * (n_steps - 1);

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t idx_x_car = 0;
size_t idx_y_car = idx_x_car + n_steps;
size_t idx_psi_car = idx_y_car + n_steps;
size_t idx_v_car = idx_psi_car + n_steps;
size_t idx_cte_car = idx_v_car + n_steps;
size_t idx_err_psi_car = idx_cte_car + n_steps;
size_t idx_a_steer = idx_err_psi_car + n_steps;
size_t idx_r_throttle = idx_a_steer + n_steps - 1;

double l_front = 2.67;
double v_car_ref = 40;
double dt = t_horizon / n_steps;

double max_a_steer = 25.0 * M_PI / 180.0;
double max_r_throttle = 1.0;

class FG_eval {

    public:

        AD<double> cte;
        AD<double> err_psi;
        AD<double> err_v_car;
        Eigen::VectorXd coeffs;

        FG_eval(Eigen::VectorXd coeffs) {
            this->coeffs = coeffs;
        }

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

        void operator()(ADvector &fg, const ADvector &x) {
            
            //******************************
            // Objective function
            //******************************
        
            cte = 0;
            for (size_t t = 0; t < n_steps; t++) {
                cte += x[idx_cte_car + t];
            }

            err_psi = 0;
            for (size_t t = 0; t < n_steps; t++) {
                err_psi += x[idx_err_psi_car + t];
            }

            err_v_car = 0;
            for (size_t t = 0; t < n_steps; t++) {
                err_v_car += x[idx_v_car + t] - v_car_ref;
            }

            fg[0] = cte + err_psi + err_v_car;

            //******************************
            // Constraints
            //******************************

            // Initial conditions
            fg[1 + idx_x_car] = x[idx_x_car];
            fg[1 + idx_y_car] = x[idx_y_car];
            fg[1 + idx_psi_car] = x[idx_psi_car];
            fg[1 + idx_v_car] = x[idx_v_car];
            fg[1 + idx_cte_car] = x[idx_cte_car];
            fg[1 + idx_err_psi_car] = x[idx_err_psi_car];

            // Vehicle model constraints
            for (size_t t = 1; t < n_steps; t++) {

                // States at t
                AD<double> x_car_0 = x[idx_x_car + t - 1];
                AD<double> y_car_0 = x[idx_y_car + t - 1];
                AD<double> psi_car_0 = x[idx_psi_car + t - 1];
                AD<double> v_car_0 = x[idx_v_car + t - 1];
                AD<double> cte_car_0 = x[idx_cte_car + t - 1];
                AD<double> err_psi_car_0 = x[idx_err_psi_car + t - 1];

                // States at t+1
                AD<double> x_car_1 = x[idx_x_car + t];
                AD<double> y_car_1 = x[idx_y_car + t];
                AD<double> psi_car_1 = x[idx_psi_car + t];
                AD<double> v_car_1 = x[idx_v_car + t];
                AD<double> cte_car_1 = x[idx_cte_car + t];
                AD<double> err_psi_car_1 = x[idx_err_psi_car + t];

                // Actuators at t
                AD<double> a_steer_0 = x[idx_a_steer + t - 1];
                AD<double> r_throttle_0 = x[idx_r_throttle + t - 1];

                CppAD::AD<double> x_car_0_squared = x_car_0 * x_car_0;
                CppAD::AD<double> x_car_0_cubed = x_car_0_squared * x_car_0;
                CppAD::AD<double> f_0 = coeffs[0] + coeffs[1] * x_car_0 + coeffs[2] * x_car_0_squared + coeffs[3] * x_car_0_cubed;
                CppAD::AD<double> psi_ref_0 = atan(coeffs[1] + 2 * coeffs[2] * x_car_0 + 3 * coeffs[3] * x_car_0_squared);

                // Setup the rest of the model constraints
                fg[1 + idx_x_car + t] = x_car_1 - (x_car_0 + v_car_0 * cos(psi_car_0) * dt);
                fg[1 + idx_y_car + t] = y_car_1 - (y_car_0 + v_car_0 * sin(psi_car_0) * dt);
                fg[1 + idx_psi_car + t] = psi_car_1 - (psi_car_0 + v_car_0 / l_front * a_steer_0 * dt);
                fg[1 + idx_v_car + t] = v_car_1 - (v_car_0 + r_throttle_0 * dt);
                fg[1 + idx_cte_car + t] = cte_car_1 - (f_0 - y_car_0 + v_car_0 * sin(err_psi_car_0) * dt);
                fg[1 + idx_err_psi_car + t] = err_psi_car_1 - (psi_car_0 - psi_ref_0 + v_car_0 / l_front * a_steer_0 * dt);

            }


        }

};


bool solve(Eigen::VectorXd states, Eigen::VectorXd coeffs) {

    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x_car = states[0];
    double y_car = states[1];
    double psi_car = states[2];
    double v_car = states[3];
    double cte_car = states[4];
    double err_psi_car = states[5];

    Dvector x(n_vars);
    Dvector xl(n_vars);
    Dvector xu(n_vars);
    for (i = 0; i < n_vars; i++) {
        // Initial value of optimization variables
        x[i] = 0.0; 
        // Lower and upper limits of optimization variables
        xl[i] = -VERY_LARGE_NUMBER;
        xu[i] = VERY_LARGE_NUMBER;
    }

    x[idx_x_car] = x_car;
    x[idx_y_car] = x_car;
    x[idx_psi_car] = psi_car;
    x[idx_v_car] = v_car;
    x[idx_cte_car] = cte_car;
    x[idx_err_psi_car] = err_psi_car;

    // for (i = idx_v_car; i < idx_cte_car; i++) {
    //     xl[i] = -max_v_car;
    //     xu[i] = max_v_car;
    // }

    for (i = idx_a_steer; i < idx_r_throttle; i++) {
        xl[i] = -max_a_steer;
        xu[i] = max_a_steer;
    }

    for (i = idx_r_throttle; i < n_vars; i++) {
        xl[i] = -max_r_throttle;
        xu[i] = max_r_throttle;
    }

    // Lower and upper constraints bounds should be zero except for initial 
    // conditions
    Dvector gl(n_constraints);
    Dvector gu(n_constraints);
    for (i = 0; i < n_constraints; i++) {
        // Lower and upper constraint bounds on optimization variables
        gl[i] = 0;
        gu[i] = 0;
    }

    gl[idx_x_car] = x_car;
    gl[idx_y_car] = y_car;
    gl[idx_psi_car] = psi_car;
    gl[idx_v_car] = v_car;
    gl[idx_cte_car] = cte_car;
    gl[idx_err_psi_car] = err_psi_car;

    gu[idx_x_car] = x_car;
    gu[idx_y_car] = y_car;
    gu[idx_psi_car] = psi_car;
    gu[idx_v_car] = v_car;
    gu[idx_cte_car] = cte_car;
    gu[idx_err_psi_car] = err_psi_car;

    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    // options
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  5\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, x, xl, xu, gl, gu, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    return ok;

}


int main() {

    Poly poly;

    Eigen::VectorXd x_waypts(8); 
    Eigen::VectorXd y_waypts(8); 
    x_waypts << 0, 5, 25, 35, 50, 75, 85, 100;
    y_waypts << 0, 1, 2,  3.5,  3.8,  4.3,  4.8,  4.9;

    poly.Fit(x_waypts, y_waypts, 3);

    double dx = 1;

    vector<double> xf;
    vector<double> yf;
    for (double x = 0.0; x <= 100; x += dx) {
        xf.push_back(x);
        yf.push_back(poly.Eval(x));
        x += dx;
    }

    Eigen::VectorXd states(6);

    states << 0.0, 0.0, 0.0, 5.0, 0.0, 0.0;

    if (solve(states, poly.coeffs)) {
        std::cout << "Success" << std::endl;
    }
    else {
        std::cout << "Fail" << std::endl;
    }

    //******************************
    // Plotting
    //******************************

    vector<double> xw;
    xw.resize(x_waypts.size());
    Eigen::VectorXd::Map(&xw[0], x_waypts.size()) = x_waypts;
    
    vector<double> yw;
    yw.resize(y_waypts.size());
    Eigen::VectorXd::Map(&yw[0], y_waypts.size()) = y_waypts;

    plt::plot(xw, yw, "o");
    plt::plot(xf, yf);
    plt::show();

    return 0;

}