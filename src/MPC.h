#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define CTE_WEIGHT 1000;
#define EPSI_WEIGHT 1000;
#define STEER_WEIGHT 5;
#define ACCEL_WEIGHT 5;
#define STEER_GAP_WEIGHT 100;
#define ACCEL_GAP_WEIGHT 5;

using namespace std;

class MPC {
 public:
  MPC();
  


  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
