#include <math.h>
#include <fstream>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Poly.h"
#include "Car.h"
#include "json.hpp"

#define _USE_MATH_DEFINES
#define VERY_LARGE_NUMBER 1.0e15

using CppAD::AD;
using std::vector;
using json = nlohmann::json;

double l_front = 2.67;
double v_car_ref = 34;

double max_a_steer = 25.0 * M_PI / 180.0;
double max_r_throttle = 1.0;

struct control{
    double a_steer;
    double g_accel;
};

struct params{
    size_t n_steps;
    size_t n_constraints;
    size_t n_vars;
    size_t n_states;
    size_t n_actuators;
    double t_horizon;
    double dt;
};

struct index_map{
    size_t x_car;
    size_t y_car;
    size_t psi_car;
    size_t v_car;
    size_t cte_car;
    size_t err_psi_car;
    size_t a_steer;
    size_t g_accel;
};

template<typename Type>
struct objective{
    Type cte;
    Type err_psi;
    Type err_v_car;
    Type mag_a_steer;
    Type mag_g_accel;
};

template<typename Type>
struct output{
    control u;
    objective<Type> obj;
};

template <typename TypeX, typename TypeObj>
TypeObj obj_fun(TypeX x, params &p, index_map &idx) {

    TypeObj obj;

    obj.cte = 0;
    obj.err_psi = 0;
    obj.err_v_car = 0;
    for (size_t t = 0; t < p.n_steps; t++) {
        obj.cte += 1e4 * pow(x[idx.cte_car + t], 2);
        obj.err_psi += 1e5 * pow(x[idx.err_psi_car + t], 2);
        obj.err_v_car += 1e0 * pow(x[idx.v_car + t] - v_car_ref, 2);
    }

    obj.mag_a_steer = 0;
    obj.mag_g_accel = 0;
    for (size_t t = 0; t < p.n_steps - 1; t++) {
        obj.mag_a_steer += 1e1 * pow(x[idx.a_steer + t], 2);
        obj.mag_g_accel += 1e1 * pow(x[idx.g_accel + t], 2);
    }

    return obj;

}


class FG_eval {

    public:

        Eigen::VectorXd coeffs;
        params p;
        index_map idx;

        FG_eval(Eigen::VectorXd coeffs, params p, index_map idx) {
            this->coeffs = coeffs;
            this->p = p;
            this->idx = idx;
        }

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

        void operator()(ADvector &fg, const ADvector &x) {
            
            //******************************
            // Objective function
            //******************************
        
            // cte = 0;
            // err_psi = 0;
            // err_v_car = 0;
            // for (size_t t = 0; t < n_steps; t++) {
            //     cte += 1e4 * pow(x[idx_cte_car + t], 2);
            //     err_psi += 1e5 * pow(x[idx_err_psi_car + t], 2);
            //     err_v_car += 1e0 * pow(x[idx_v_car + t] - v_car_ref, 2);
            // }

            // mag_a_steer = 0;
            // mag_g_accel = 0;
            // for (size_t t = 0; t < n_steps - 1; t++) {
            //     mag_a_steer += 0e1 * pow(x[idx_a_steer + t], 2);
            //     mag_g_accel += 0e1 * pow(x[idx_g_accel + t], 2);
            // }

            objective<AD<double>> obj;
            obj = obj_fun<ADvector, objective<AD<double>>>(x, p, idx);

            fg[0] = obj.cte + 
                    obj.err_psi + 
                    obj.err_v_car + 
                    obj.mag_a_steer + 
                    obj.mag_g_accel;

            //******************************
            // Constraints
            //******************************

            // Initial conditions
            fg[1 + idx.x_car] = x[idx.x_car];
            fg[1 + idx.y_car] = x[idx.y_car];
            fg[1 + idx.psi_car] = x[idx.psi_car];
            fg[1 + idx.v_car] = x[idx.v_car];
            fg[1 + idx.cte_car] = x[idx.cte_car];
            fg[1 + idx.err_psi_car] = x[idx.err_psi_car];

            // Vehicle model constraints
            for (size_t t = 1; t < p.n_steps; t++) {

                // States at t
                AD<double> x_car_0 = x[idx.x_car + t - 1];
                AD<double> y_car_0 = x[idx.y_car + t - 1];
                AD<double> psi_car_0 = x[idx.psi_car + t - 1];
                AD<double> v_car_0 = x[idx.v_car + t - 1];
                AD<double> cte_car_0 = x[idx.cte_car + t - 1];
                AD<double> err_psi_car_0 = x[idx.err_psi_car + t - 1];

                // States at t+1
                AD<double> x_car_1 = x[idx.x_car + t];
                AD<double> y_car_1 = x[idx.y_car + t];
                AD<double> psi_car_1 = x[idx.psi_car + t];
                AD<double> v_car_1 = x[idx.v_car + t];
                AD<double> cte_car_1 = x[idx.cte_car + t];
                AD<double> err_psi_car_1 = x[idx.err_psi_car + t];

                // Actuators at t
                AD<double> a_steer_0 = x[idx.a_steer + t - 1];
                AD<double> r_throttle_0 = x[idx.g_accel + t - 1];

                CppAD::AD<double> x_car_0_squared = x_car_0 * x_car_0;
                CppAD::AD<double> x_car_0_cubed = x_car_0_squared * x_car_0;
                CppAD::AD<double> f_0 = coeffs[0] + coeffs[1] * x_car_0 + coeffs[2] * x_car_0_squared + coeffs[3] * x_car_0_cubed;
                CppAD::AD<double> psi_ref_0 = atan(coeffs[1] + 2 * coeffs[2] * x_car_0 + 3 * coeffs[3] * x_car_0_squared);

                // Setup the rest of the model constraints
                fg[1 + idx.x_car + t] = x_car_1 - (x_car_0 + v_car_0 * cos(psi_car_0) * p.dt);
                fg[1 + idx.y_car + t] = y_car_1 - (y_car_0 + v_car_0 * sin(psi_car_0) * p.dt);
                fg[1 + idx.psi_car + t] = psi_car_1 - (psi_car_0 + v_car_0 / l_front * a_steer_0 * p.dt);
                fg[1 + idx.v_car + t] = v_car_1 - (v_car_0 + r_throttle_0 * p.dt);
                fg[1 + idx.cte_car + t] = cte_car_1 - (f_0 - y_car_0 + v_car_0 * sin(err_psi_car_0) * p.dt);
                fg[1 + idx.err_psi_car + t] = err_psi_car_1 - (psi_car_0 - psi_ref_0 + v_car_0 / l_front * a_steer_0 * p.dt);

            }


        }

};


output<double> solve(Eigen::VectorXd states, Eigen::VectorXd coeffs, params p, index_map idx) {

    output<double> out; 

    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x_car = states[0];
    double y_car = states[1];
    double psi_car = states[2];
    double v_car = states[3];
    double cte_car = states[4];
    double err_psi_car = states[5];

    Dvector x(p.n_vars);
    Dvector xl(p.n_vars);
    Dvector xu(p.n_vars);
    for (i = 0; i < p.n_vars; i++) {
        // Initial value of optimization variables
        x[i] = 0.0; 
        // Lower and upper limits of optimization variables
        xl[i] = -VERY_LARGE_NUMBER;
        xu[i] = VERY_LARGE_NUMBER;
    }

    x[idx.x_car] = x_car;
    x[idx.y_car] = x_car;
    x[idx.psi_car] = psi_car;
    x[idx.v_car] = v_car;
    x[idx.cte_car] = cte_car;
    x[idx.err_psi_car] = err_psi_car;

    // for (i = idx.v_car; i < idx.cte_car; i++) {
    //     xl[i] = -max_v_car;
    //     xu[i] = max_v_car;
    // }

    for (i = idx.a_steer; i < idx.g_accel; i++) {
        xl[i] = -max_a_steer;
        xu[i] = max_a_steer;
    }

    for (i = idx.g_accel; i < p.n_vars; i++) {
        xl[i] = -max_r_throttle;
        xu[i] = max_r_throttle;
    }

    // Lower and upper constraints bounds should be zero except for initial 
    // conditions
    Dvector gl(p.n_constraints);
    Dvector gu(p.n_constraints);
    for (i = 0; i < p.n_constraints; i++) {
        // Lower and upper constraint bounds on optimization variables
        gl[i] = 0;
        gu[i] = 0;
    }

    gl[idx.x_car] = x_car;
    gl[idx.y_car] = y_car;
    gl[idx.psi_car] = psi_car;
    gl[idx.v_car] = v_car;
    gl[idx.cte_car] = cte_car;
    gl[idx.err_psi_car] = err_psi_car;

    gu[idx.x_car] = x_car;
    gu[idx.y_car] = y_car;
    gu[idx.psi_car] = psi_car;
    gu[idx.v_car] = v_car;
    gu[idx.cte_car] = cte_car;
    gu[idx.err_psi_car] = err_psi_car;

    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs, p, idx);

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

    // Evaluate constraints with solution
    objective<double> obj;
    obj = obj_fun<Dvector, objective<double>>(solution.x, p, idx);

    out.u.a_steer = solution.x[idx.a_steer];
    out.u.g_accel = solution.x[idx.g_accel];
    out.obj.cte = obj.cte;
    out.obj.err_psi = obj.err_psi;
    out.obj.err_v_car = obj.err_v_car;
    out.obj.mag_a_steer = obj.mag_a_steer;
    out.obj.mag_g_accel = obj.mag_g_accel;

    return out;

}


int main(int argc, char *argv[]) {

    // Read json file with params
    std::ifstream param_file;
    param_file.open(argv[1]);
    json user_params;
    param_file >> user_params;
    param_file.close();

    params p;

    p.t_horizon = 1;
    p.n_steps = 10;
    p.dt = p.t_horizon / p.n_steps;
    p.n_states = 6;
    p.n_actuators = 2;
    p.n_constraints = p.n_steps * p.n_states; 
    p.n_vars = p.n_states * p.n_steps + p.n_actuators * (p.n_steps - 1);

    // The solver takes all the state variables and actuator
    // variables in a singular vector. Thus, we should to establish
    // when one variable starts and another ends to make our lifes easier.
    
    index_map idx;
    idx.x_car = 0;
    idx.y_car = idx.x_car + p.n_steps;
    idx.psi_car = idx.y_car + p.n_steps;
    idx.v_car = idx.psi_car + p.n_steps;
    idx.cte_car = idx.v_car + p.n_steps;
    idx.err_psi_car = idx.cte_car + p.n_steps;
    idx.a_steer = idx.err_psi_car + p.n_steps;
    idx.g_accel = idx.a_steer + p.n_steps - 1;

    Poly poly;
    output<double> o;

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

    // Simulation
    Car car(l_front);
    car.v = 30;
    car.psi = 0 * M_PI/180.0;

    vector<double> x_car;
    vector<double> y_car;
    vector<double> psi_car;
    vector<double> v_car;
    vector<double> a_steer;
    vector<double> g_accel;
    vector<double> cte, err_psi, err_v_car, mag_a_steer, mag_g_accel;

    size_t n_sim_steps = 50;
    double dt_sim = 0.1;

    Eigen::VectorXd states(6);
    states << 0.0, 0.0, 0.0, 5.0, 0.0, 0.0;
    for (size_t i = 0; i < n_sim_steps; i++) {

        // Update state vector
        car.cte = (poly.Eval(car.x) - car.y) * cos(car.psi);
        car.err_psi = car.psi - atan(poly.Diff(car.x));
        states << car.x, car.y, car.psi, car.v, car.cte, car.err_psi;

        o = solve(states, poly.coeffs, p, idx);
        car.Step(o.u.a_steer, o.u.g_accel, dt_sim);
        
        // Logging
        x_car.push_back(car.x);
        y_car.push_back(car.y);
        psi_car.push_back(car.psi);
        v_car.push_back(car.v);
        a_steer.push_back(o.u.a_steer);
        g_accel.push_back(o.u.g_accel);
        cte.push_back(o.obj.cte);
        err_psi.push_back(o.obj.err_psi);
        err_v_car.push_back(o.obj.err_v_car);
        mag_a_steer.push_back(o.obj.mag_a_steer);
        mag_g_accel.push_back(o.obj.mag_g_accel); 

    }

    vector<double> xw;
    xw.resize(x_waypts.size());
    Eigen::VectorXd::Map(&xw[0], x_waypts.size()) = x_waypts;
    
    vector<double> yw;
    yw.resize(y_waypts.size());
    Eigen::VectorXd::Map(&yw[0], y_waypts.size()) = y_waypts;

    //******************************
    // JSON Results file
    //******************************
    std::ofstream json_file;
    json_file.open("json_results.txt");

    json j_results;
    
    j_results["waypoints"]["nodes"]["x"] = xw;
    j_results["waypoints"]["nodes"]["y"] = yw;
    j_results["waypoints"]["fit"]["x"] = xf;
    j_results["waypoints"]["fit"]["y"] = yf;
    j_results["sim"]["xCar"] = x_car;
    j_results["sim"]["yCar"] = y_car;
    j_results["sim"]["vCar"] = v_car;
    j_results["sim"]["psiCar"] = psi_car;
    j_results["sim"]["aSteer"] = a_steer;
    j_results["sim"]["gAccel"] = g_accel;
    j_results["sim"]["cte"] = cte;
    j_results["sim"]["err_psiCar"] = err_psi;
    j_results["sim"]["err_vCar"] = err_v_car;
    j_results["sim"]["mag_aSteer"] = mag_a_steer;
    j_results["sim"]["mag_gAccel"] = mag_g_accel;

    json_file << j_results;
    json_file.close();

    return 0;

}