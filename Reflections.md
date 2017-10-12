# Project MPC Control - Reflections
## Sai Prateek Medi

This was a very interesting project to work on encompassing much of the course content be it map to vehicle transform (particle filter), kinematic models, understanding CTE (PID Control) and finally, prediction horizons with Model Predictive Controllers. I'm happy to have learned from these projects and additionally picked up C++ to a suitable competency along the way.


## The Model:

The Model Predictive Control system that was implemented here works on the Udacity Self Driving Car Simulator that has a map involving a number of obstacles and turns. The system uses the concept of a prediction horizon that looks forward into the track.

### State Variables:
This particular implementation uses 4 state variables, X coordinates, Y Coordinates, Psi - a bearing angle, and Velocity. The coordinate system used by the MPC model refers to forward X as the heading of the car, with Y being lateral position on the track, Psi being the turn angle, and V the velocity the car is heading.

### Coordinate System Transformation: 
The simulator we used with the model refers to a Map coordinate system, whereas the MPC uses coordinatets from the vehicle's coordinate system. Hence, when feedback is given from the simulator, the coordinate points have to be transformed to the Vehicle's POV/Coordinate System for MPC using these equations:
				
**********************************************					
		Xm = Xp + cos(psi) * Xc - sin(theta) * Yc 
		Ym = Yp + sin(psi) * Xc + cos(theta) * Yc 
**********************************************	
And using the simulator's documentation, the rotation was -90 hence the angle (psi) from the simulator was negated.

### Actuators:

The actuators that were used are Delta - which is used to modify Psi as well as the error term for Psi.
The second actuator used was acceleration. Both of these actuators are used as inputs to the MPC control with weighting that influences their impact to the MPC. 

### Latency:
The MPC allows for the modelling of latency. This implementation experimented with latency from 50 milliseconds to 200 milliseconds, settling on using 100 milliseconds.
This latency indeed influences the state variables based on these equations:
 
Latency factored into the kinematic model will serve to help correct the 
predicted state variables such that this unpreparedness can be overcome.
**********************************************  
        Necessary equations:
        v_corr = v0 + a*latency
        x_corr = v*latency
        psi_corr =  (v/Lf) * psi * latency
        cte_corr = cte + v*sin(epsi)*latency
        epsi_corr = epsi + (v/Lf) *psi *latency
        y in car's reference will always be 0 (no car sideways velocity possible in cars)

### Polynomial Fitting:

The simulator returns a series of coordinate points that point towards the expected heading of the car along with values such as current speed, acceleration, turn-rate, cross-track-error, turn-error among other metrics. A key process in this implementation is to use the coordinates given and fit a polynomial over the points. Before this, the points are to be transformed to vehicle coordinates.

The polynomial fitting is key to the Model because the Model is given a Predictive Horizon upon the trajectory of the Polynomial with respect to the car's current and future locations.

After the state variables are processed according the latency above, the state variables and polynomial coefficients are passed to the Model Predictive Control's system.


### MPC System:

#### Timestep Lenth and Elapsed Duration N & dt:

The MPC builds a vector of state variables across a Prediction Horizon. This implementation uses the number of steps (N) to 15, and timesteps intervals (dt) as .15,  150 milliseconds. This dictates the prediction horizon to be ~ 2.25 seconds in the future. Previous values included N=25, dt = .1 and .2 among others. Larger timesteps and time windows make the solution more computationally intensive. The Reference speed chosen was set to average around 15 m/s for this model to sucessfully complete the track. 

#### Cost Function:
An important feature in the model is to use a cost function that is told the initial state of the system, the boundary of values that can exist for each state variable, and the constraints necessary to contain the system.
Equations are used in this system to determine predictions on the Prediction Horizon based on kinematic equations.
  
Here is an example of equations used to create constraints on the system that creates a solution optimizing the lowest error predictions.


		AD<double> f0 = coeffs[3]*CppAD::pow(x0,3) + coeffs[2]*CppAD::pow(x0,2) + coeffs[1]*x0 + coeffs[0];
		AD<double> psides0  = CppAD:: atan(3*coeffs[3] * CppAD::pow(x0,2) + 2*coeffs[2]*x0 + coeffs[1]);
		// calculating cost difference from future state t+1 minus predicted from t.	
		// x
		fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt); 
		// y		
		fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt); 
		// psi		
		fg[1 + psi_start + t] = psi1 - (psi0 + (v0/Lf) * delta * dt);
		// v
		fg[1 + v_start + t] = v1 - (v0 + a*dt);	
		// cte
		fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));	
		// epsi		
		fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta/Lf*dt);
  



### Weighting the cost function:
A significant amount of testing and design was used to determine optimal weights upon the elements of the cost function. Aspects of the cost function can be modified to push for a solution optimized for a given task:

Many different individual constraint/cost portions were up-weighted and down-weighted to determine the efficacy on the MPC.
The solution implemented upweights Orientation error, and change in orientation error to best help the model guide the vehicle on the track with an average speed of 15 m/s.

		fg[0] +=  100*CppAD::pow(vars[cte_start + t],2);   // cross track error
		fg[0] +=  3000*CppAD::pow(vars[epsi_start + t],2);  // orientation error
		fg[0] +=  500*CppAD::pow(vars[v_start + t] - ref_v,2);     // velocity components

		fg[0] +=  2000*CppAD::pow(vars[delta_start + t],2);
		fg[0] +=  500*CppAD::pow(vars[a_start + t],2);

		fg[0] +=  13000*CppAD::pow((vars[delta_start + t + 1] - vars[delta_start+t]),2);		
		fg[0] +=  100*CppAD::pow((vars[a_start + t + 1] - vars[a_start + t]),2);	






## Conclusion and Future Works:
In closing, this was a very informative and incredible project to work on. There are many ways to tweak and optimize the system to achieve higher speeds. This would involve tweaking the weights on the system better along with modifying the Prediction Horizon at the same time.




