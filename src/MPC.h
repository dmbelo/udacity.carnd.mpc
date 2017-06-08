#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Poly.h"

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::vector<double> Solve(Eigen::VectorXd state, Poly poly);
  std::vector<double> xv_opt_traj;
  std::vector<double> yv_opt_traj;
};

#endif /* MPC_H */
