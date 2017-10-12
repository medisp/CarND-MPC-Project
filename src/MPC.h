#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

//using namespace std; this single line will ruin any compiling!! my god json package
//https://github.com/nlohmann/json/issues/590 
// now in every time vector is used, must be prefaced with std:: bc namespace causes errors.

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
