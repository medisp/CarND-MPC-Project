#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
const size_t N = 15;
const double dt = 0.15;  // first attempt N=25, dt=0.2 hence T = 5 

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
const double Lf = 2.67;

const double ref_v = 15 * 0.44704; // mph to m/s conversion
// Creating the indices to refer to variable and actuators within the single vector for solver

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N -1 ; // flag



class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
	
	// initially set to 0
	fg[0] = 0;
		// initial weights: 1000,800,200,400,400,500,500
	// cost based on reference state error 
	//std::cout << "FG - cte error ref states " << std::endl;
	for (int t = 0; t < N; t++) {
		fg[0] +=  100*CppAD::pow(vars[cte_start + t],2);   // cross track error
		fg[0] +=  3000*CppAD::pow(vars[epsi_start + t],2);  // orientation error
		fg[0] +=  500*CppAD::pow(vars[v_start + t] - ref_v,2);     // velocity components
	} // end forloop reference error

	//std::cout << "FG - actuators error ref states " << std::endl;
	//cost based on actuators delta and acceleration
	for ( int t = 0; t < N-1; t++) {                  // N-1 due to less values
		fg[0] +=  2000*CppAD::pow(vars[delta_start + t],2);
		fg[0] +=  500*CppAD::pow(vars[a_start + t],2);
	}

	//std::cout << "FG - actuator seq diff part " << std::endl;
	//cost based on difference sequential events (sort of like Kd)
	for ( int t = 0; t < N-2; t++) {
				// increasing this to keep sequential steering values closer (minimizing this) 
		fg[0] +=  13000*CppAD::pow((vars[delta_start + t + 1] - vars[delta_start+t]),2);		
		fg[0] +=  100*CppAD::pow((vars[a_start + t + 1] - vars[a_start + t]),2);	
	}

	//std::cout << "FG - initial constraints " << std::endl;
	// setting initial constraints
	fg[1 + x_start] = vars[x_start];
	fg[1 + y_start] = vars[y_start];
	fg[1 + psi_start] = vars[psi_start];
	fg[1 + v_start] = vars[v_start];
	fg[1 + cte_start] = vars[cte_start];
	fg[1 + epsi_start] = vars[epsi_start];

	// setting the total constraints
	//std::cout << "FG - setting total constraints " << std::endl;	
	for(int t = 1; t < N; t++) {  // using index 1 bc fg:cost at 0
		
		// xt and xt+1
		AD<double> x1 = vars[x_start + t];
		AD<double> x0 = vars[x_start + t - 1];

		// yt and yt+1
		AD<double> y1 = vars[y_start + t];
		AD<double> y0 = vars[y_start + t - 1];

		// psi_t and psi_t+1		
		AD<double> psi1 = vars[psi_start + t];
		AD<double> psi0 = vars[psi_start + t - 1];

		// vt and vt+1
		AD<double> v1 =  vars[v_start + t];
		AD<double> v0 =  vars[v_start + t - 1];
			
		// cte_t and cte_t+1
		AD<double> cte1 = vars[cte_start + t];
		AD<double> cte0 = vars[cte_start + t - 1];

		// epsi_t and epsi_t+1
		AD<double> epsi1 = vars[epsi_start + t];		
		AD<double> epsi0 = vars[epsi_start + t - 1];

		// only need actuators at current time
		AD<double> delta = vars[delta_start + t - 1];
		AD<double> a = vars[a_start + t - 1];

		//f0 psides0
		AD<double> f0 = coeffs[3]*CppAD::pow(x0,3) + coeffs[2]*CppAD::pow(x0,2) + coeffs[1]*x0 + coeffs[0]; // vars[delta_start + t - 1];
		AD<double> psides0  = CppAD:: atan(3*coeffs[3] * CppAD::pow(x0,2) + 2*coeffs[2]*x0 + coeffs[1]); //vars[delta_start + t - 1];  // replace with desired fit poly
								// derivative term -> arctan
		// calculating cost difference from future state t+1 minus predicted from t.	
		// x
		fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);  //accidental mult		
		// y		
		fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);  //accidental mult
		// psi		
		fg[1 + psi_start + t] = psi1 - (psi0 + (v0/Lf) * delta * dt);
		// v
		fg[1 + v_start + t] = v1 - (v0 + a*dt);
		
		// cte
		fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
		
		// epsi		
		fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta/Lf*dt);



	} // end forloop t
		

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9  

	// 25 time stamps, 4*25 + 2 *24 = 148 // N * 6 + (N-1) *2
	size_t n_vars = N * 6 + (N - 1) * 2;

	
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6; // due to number of variables and total time steps for each

	//AD<double x = x0[0]	
	
  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.

  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
	std::cout << "setting initial state " << std::endl;
	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];	

	std::cout << "setting vars start " << std::endl;	
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;


  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

	std::cout << "setting upper/lower bounds for vars" << std::endl;
	/*
	for( int i = 0; i < n_vars; i++) {
		// min and max values for non_actuators			
		if ( i < delta_start) {
			vars_lowerbound[i] = -1.0e19;		
			vars_upperbound[i] =  1.0e19;
		} // end if-vars
	
		// actuators		
		// delta	
		if (i >= delta_start && i < a_start) {
			vars_lowerbound[i] = -0.436332;		
			vars_upperbound[i] =  0.436332;
		} // end if_i delta
		
		// alpha
		else if (i >= a_start && i < n_vars) {
			
			vars_lowerbound[i] = -1.0;
			vars_upperbound[i] =  1.0;	
		} // end if_i alpha

	} // end forloop i	
	*/
	for( int i = 0; i < delta_start; i++) {
		vars_lowerbound[i] = -1.0e19;		
		vars_upperbound[i] =  1.0e19;
	}
	for( int i = delta_start; i < a_start; i++) {
		vars_lowerbound[i] = -0.436332;		
		vars_upperbound[i] =  0.436332;
	}
	for( int i = a_start; i < n_vars; i++) {
		vars_lowerbound[i] = -1.0;		
		vars_upperbound[i] =  1.0;
	}


	std::cout << "setting upper and lower constraints " << std::endl;
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  	
	for (int i = 0; i < n_constraints; i++) {
    	constraints_lowerbound[i] = 0;
    	constraints_upperbound[i] = 0;
  	} // end forloop constraints

	constraints_lowerbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi; // this accidentally being psi caused a world of debugging hurt

	constraints_upperbound[x_start] = x;
	constraints_upperbound[y_start] = y;
	constraints_upperbound[psi_start] = psi;
	constraints_upperbound[v_start] = v;
	constraints_upperbound[cte_start] = cte;
	constraints_upperbound[epsi_start] = epsi;
	
	std::cout << "sending to fg_eval(coeffs) " << std::endl;	
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

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


std::cout << "cppad ipopt creating place for solution " << std::endl;
  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

std::cout << "cppad ipopt tryin to solve solution " << std::endl;
  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

std::cout << "checking solution values " << std::endl;
  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

std::cout << "cost value " << std::endl;
  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
	// returning first 10 values:
	//std::cout << "trying to return solution " << std::endl;  
	std::vector<double> mpc_vec;//= std::vector<double>(solution.x.size());
	
	mpc_vec.push_back(solution.x[delta_start]);
	mpc_vec.push_back(solution.x[a_start]);	

std::cout << "trying to loop through x and y start " << std::endl;  	
	for (int k = 0; k < N; k++) {
		mpc_vec.push_back(solution.x[x_start+k]);
	}
	for (int k = 0; k < N; k++) {
		mpc_vec.push_back(solution.x[y_start+k]);
	}


	std::cout << "size of solution.x " << solution.x.size() << std::endl;

  return mpc_vec;

}


/*

{solution.x[x_start + 1],solution.x[y_start + 1],
			solution.x[psi_start + 1], solution.x[v_start + 1],
			solution.x[cte_start + 1], solution.x[epsi_start + 1],
			solution.x[delta_start], solution.x[a_start]}

*/

