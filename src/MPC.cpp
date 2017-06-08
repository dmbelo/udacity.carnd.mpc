#include "MPC.h"
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.2;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t idx_x_car = 0;
size_t idx_y_car = idx_x_car + N;
size_t idx_psi_car = idx_y_car + N;
size_t idx_v_car = idx_psi_car + N;
size_t idx_cte_car = idx_v_car + N;
size_t idx_err_psi_car = idx_cte_car + N;
size_t idx_a_steer = idx_err_psi_car + N;
size_t idx_r_throttle = idx_a_steer + N - 1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// a was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double a = 2.67;

// The reference velocity
double v_car_ref = 30;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  // Poly poly;
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    
    // Cost function
    fg[0] = 0;
    
    // Track the reference trajectory
    for (unsigned int t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[idx_cte_car + t], 2);
      fg[0] += CppAD::pow(vars[idx_err_psi_car + t], 2);
      fg[0] += CppAD::pow(vars[idx_v_car + t] - v_car_ref, 2);
    }

    // Minimize actuator magnitude
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 500 * CppAD::pow(vars[idx_a_steer + t], 2);
      fg[0] += 100 * CppAD::pow(vars[idx_r_throttle + t], 2);
    }

    // Minimize actuator derivatives
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 500 * CppAD::pow(vars[idx_a_steer + t + 1] - vars[idx_a_steer + t], 2);
      fg[0] += 50 * CppAD::pow(vars[idx_r_throttle + t + 1] - vars[idx_r_throttle + t], 2);
    }

    // Setup Constraints

    // Initial constraints
    fg[1 + idx_x_car] = vars[idx_x_car];
    fg[1 + idx_y_car] = vars[idx_y_car];
    fg[1 + idx_psi_car] = vars[idx_psi_car];
    fg[1 + idx_v_car] = vars[idx_v_car];
    fg[1 + idx_cte_car] = vars[idx_cte_car];
    fg[1 + idx_err_psi_car] = vars[idx_err_psi_car];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t
      CppAD::AD<double> x_car_0 = vars[idx_x_car + t - 1];
      CppAD::AD<double> y_car_0 = vars[idx_y_car + t - 1];
      CppAD::AD<double> psi_car_0 = vars[idx_psi_car + t - 1];
      CppAD::AD<double> v_car_0 = vars[idx_v_car + t - 1];
      // CppAD::AD<double> cte0 = vars[idx_cte_car + t - 1];
      CppAD::AD<double> err_psi_car_0 = vars[idx_err_psi_car + t - 1];

      // The state at time t+1
      CppAD::AD<double> x_car_1 = vars[idx_x_car + t];
      CppAD::AD<double> y_car_1 = vars[idx_y_car + t];
      CppAD::AD<double> psi_car_1 = vars[idx_psi_car + t];
      CppAD::AD<double> v_car_1 = vars[idx_v_car + t];
      CppAD::AD<double> cte_car_1 = vars[idx_cte_car + t];
      CppAD::AD<double> err_psi_car_1 = vars[idx_err_psi_car + t];

      // Actuations at time t
      CppAD::AD<double> a_steer_0 = vars[idx_a_steer + t - 1];
      CppAD::AD<double> r_throttle_0 = vars[idx_r_throttle + t - 1];

      // Pre-calculate for readability
      // CppAD::AD<double> f_0 = poly.Eval(x_car_0);
      // CppAD::AD<double> psi_ref_0 = CppAD::atan(poly.Diff(x_car_0));
      // Explicit 3rd order polynomial case
      CppAD::AD<double> x_car_0_squared = x_car_0 * x_car_0;
      CppAD::AD<double> x_car_0_cubed = x_car_0_squared * x_car_0;
      CppAD::AD<double> f_0 = coeffs[0] + coeffs[1] * x_car_0 + coeffs[2] * x_car_0_squared + coeffs[3] * x_car_0_cubed;
      CppAD::AD<double> psi_ref_0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x_car_0 + 3 * coeffs[3] * x_car_0_squared);
      // Explicit 1st order polynomial case
      // CppAD::AD<double> f_0 = poly.coeffs[0] + poly.coeffs[1] * x_car_0;
      // CppAD::AD<double> psi_ref_0 = CppAD::atan(poly.coeffs[1]);
      

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.

      // Setup the rest of the model constraints
      fg[1 + idx_x_car + t] = x_car_1 - (x_car_0 + v_car_0 * CppAD::cos(psi_car_0) * dt);
      fg[1 + idx_y_car + t] = y_car_1 - (y_car_0 + v_car_0 * CppAD::sin(psi_car_0) * dt);
      fg[1 + idx_psi_car + t] = psi_car_1 - (psi_car_0 + v_car_0 / a * a_steer_0 * dt);
      fg[1 + idx_v_car + t] = v_car_1 - (v_car_0 + r_throttle_0 * dt);
      fg[1 + idx_cte_car + t] = cte_car_1 - (f_0 - y_car_0 + v_car_0 * CppAD::sin(err_psi_car_0) * dt);
      fg[1 + idx_err_psi_car + t] = err_psi_car_1 - (psi_car_0 - psi_ref_0 + v_car_0 / a * a_steer_0 * dt);

    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(Eigen::VectorXd states, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x_car = states[0];
  double y_car = states[1];
  double psi_car = states[2];
  double v_car = states[3];
  double cte_car = states[4];
  double err_psi_car = states[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
    // std::cout << vars[i] << std::endl;
  }

  vars[idx_x_car] = x_car;
  vars[idx_y_car] = y_car;
  vars[idx_psi_car] = psi_car;
  vars[idx_v_car] = v_car;
  vars[idx_cte_car] = cte_car;
  vars[idx_err_psi_car] = err_psi_car;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  // TODO what about velocity ... we know that's not going to be -ve
  // TODO what about psi... is it wrapped?
  for (int i = 0; i < idx_a_steer; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = idx_a_steer; i < idx_r_throttle; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = idx_r_throttle; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[idx_x_car] = x_car;
  constraints_lowerbound[idx_y_car] = y_car;
  constraints_lowerbound[idx_psi_car] = psi_car;
  constraints_lowerbound[idx_v_car] = v_car;
  constraints_lowerbound[idx_cte_car] = cte_car;
  constraints_lowerbound[idx_err_psi_car] = err_psi_car;

  constraints_upperbound[idx_x_car] = x_car;
  constraints_upperbound[idx_y_car] = y_car;
  constraints_upperbound[idx_psi_car] = psi_car;
  constraints_upperbound[idx_v_car] = v_car;
  constraints_upperbound[idx_cte_car] = cte_car;
  constraints_upperbound[idx_err_psi_car] = err_psi_car;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  1\n";
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
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  // std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  // Load solution trajectory into traj_x and traj_y for visualization.
  xv_opt_traj.clear();
  yv_opt_traj.clear();
  for (unsigned int i=0; i < N - 1; i++) {
    xv_opt_traj.push_back(solution.x[idx_x_car + i + 1]);
    yv_opt_traj.push_back(solution.x[idx_y_car + i + 1]);
  }

  return {solution.x[idx_x_car + 1],   solution.x[idx_y_car + 1],
          solution.x[idx_psi_car + 1], solution.x[idx_v_car + 1],
          solution.x[idx_cte_car + 1], solution.x[idx_err_psi_car + 1],
          solution.x[idx_a_steer],   solution.x[idx_r_throttle]};

}
