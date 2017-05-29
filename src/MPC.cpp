#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Setting the timestep length and duration
size_t N = 15;
double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
// Lf is the distance between the vehicle's front and its barycenter. 
// Lf and velocity determinate the turning radius of the vehicle.
const double Lf = 2.67;

// Defining position of values (based on N) given that
// all state and actuator variables are chained in a vector 
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 45;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  double v;
  FG_eval(Eigen::VectorXd coeffs, double v) { this->coeffs = coeffs, this->v = v; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implementing MPC:
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
	
	// Setting the cost in the first element of vector fg
	fg[0] = 0;
	
	// Building the cost function in 3 steps, the single elements are added
	// and, possible, multiplied by a factor, in order to customize MPC 
	
	// 1. The part of the cost based on the reference state.
	for (int i = 0; i < N; i++) {
		fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte, 2);
		fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi, 2) ;
		fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2) ;
	}
	
	// 2. Minimizing the use of actuators.
	// This progressive penalization minimizes steering at higher speeds
	// It takes the actual speed and elaborates a penalty multiplier
	// for steering and sequential steering
	double delta_start_factor = std::max(double(1), 18.75 * v * v - 1308.9 * v + 23893);

	// This penalization prevents the car from keeping on braking at low speed
	double delta_a_start = 1.5;
	
	for (int i = 0; i < N - 1; i++) {
		fg[0] += CppAD::pow(vars[delta_start + i], 2) * delta_start_factor;
		fg[0] += CppAD::pow(vars[a_start + i], 2) * delta_a_start;
	}
	
	// 3. Minimizing the value gap between sequential actuations.
	for (int i = 0; i < N - 2; i++) {
		fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2) * delta_start_factor;
		fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2) * delta_a_start;
	}
	
	// Setting up constraints starting from actual state
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`. This bumps up the position of all the other values.
    fg[x_start + 1]     = vars[x_start];
    fg[y_start + 1]     = vars[y_start];
    fg[psi_start + 1]   = vars[psi_start];
    fg[v_start + 1]     = vars[v_start];
    fg[cte_start + 1]   = vars[cte_start];
    fg[epsi_start + 1]  = vars[epsi_start];

    // Elaborating the update of the constraints
    for (int i = 0; i < N - 1; i++) {
	
      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];
	  
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];
	  
	  // Only considering the actuation at time t.
	  AD<double> delta0 = vars[delta_start + i];
	  AD<double> a0 = vars[a_start + i];
	  AD<double> f0 = coeffs[0];
	  AD<double> psides0 = 0;
	  for (int j = 1; j < coeffs.size(); j++) {
			f0 += coeffs[j] * CppAD::pow(x0, j);
			psides0 +=  coeffs[j] * CppAD::pow(x0, j-1) * j;
	  }
	  psides0 = CppAD::atan(psides0);
	  
	  // Estimating the difference between observed and updated state variables at t+2	  
	  // Recalling the update equations used to compute the state of the car  
	  // at the next time step, based on the state at the current time step:
	  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
	  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
	  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
	  // v_[t+1] = v[t] + a[t] * dt
	  // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
	  // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
	  fg[x_start + i + 2]    = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
	  fg[y_start + i + 2]    = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
	  fg[psi_start + i + 2]  = psi1 - (psi0 + v0 * delta0 / Lf * dt);
	  fg[v_start + i + 2]    = v1 - (v0 + a0 * dt);
	  fg[cte_start + i + 2]  = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
	  fg[epsi_start + i + 2] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
	}
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,
vector<double> &mpc_x_vals, vector<double> &mpc_y_vals) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];
  
  // Setting the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = (6 * N) + (2 * (N - 1)) ;
  // Setting the number of constraints
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Setting lower and upper limits for variables.
  
  // simply allowing all state any value
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // bounding steering between +/- 25 degrees expressed in radiants 
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -25 * M_PI / 180;
    vars_upperbound[i] = 25 * M_PI / 180;
  }

  // bounding throttle between +/- 1
  for (int i = a_start; i < n_vars; i++) {
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
  
  // Fixing the constraints
  constraints_lowerbound[x_start]     = x;
  constraints_lowerbound[y_start]     = y;
  constraints_lowerbound[psi_start]   = psi;
  constraints_lowerbound[v_start]     = v;
  constraints_lowerbound[cte_start]   = cte;
  constraints_lowerbound[epsi_start]  = epsi;

  constraints_upperbound[x_start]     = x;
  constraints_upperbound[y_start]     = y;
  constraints_upperbound[psi_start]   = psi;
  constraints_upperbound[v_start]     = v;
  constraints_upperbound[cte_start]   = cte;
  constraints_upperbound[epsi_start]  = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, v);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
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
  std::cout << "Cost " << cost << std::endl;

  // Returning the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  
  // display a line projection in green
  mpc_x_vals.resize(N);
  mpc_y_vals.resize(N);
  
  for (int i = 0; i < N; i++) {
	  mpc_x_vals[i] = solution.x[x_start + i];
	  mpc_y_vals[i] = solution.x[y_start + i];
  }
    
  // averaging a certan number of predictions in order 
  // to achieve stability when latency is a problem
  double steer_value = 0.0;
  double throttle_value = 0.0;
  
  double look_forward = 5;
  for (int i = 0; i < look_forward; i++) {
	  steer_value += solution.x[delta_start + i] / look_forward;
	  throttle_value += solution.x[a_start+1] / look_forward;
  }
  
  return {steer_value, throttle_value};
}
