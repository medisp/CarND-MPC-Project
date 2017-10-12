#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;



using namespace std;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::vector<double> ptsx = j[1]["ptsx"];
          std::vector<double> ptsy = j[1]["ptsy"];
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v = j[1]["speed"];
          const double acc = j[1]["throttle"];
          const double delta = j[1]["steering_angle"]; 
          /*
          * According to Udacity Project walkthrough video, simulator gives speed in mph
          * and metric units are necessary for MPC
          * Latency can also be factored in
          */
           const double conversionfactor =  0.44704; //thanks google converter
           const int num_pts = ptsx.size();   
           double v_mps = v * conversionfactor;

			/* The simulator gives back ptsx and ptsy above in line 88,89
				seems like polynomial fitting is best to do with this data						  	
				and then send it to the MPC controller with state values.

				**********************************************

				But first the orientation from the simulator is in map coordinates 
				and they must be converted to vehicle coordinates:
				Untrivially enough, this is the opposite of the particle filter to map 
				
				**********************************************					
				Xm = Xp + cos(psi) * Xc - sin(theta) * Yc 
				Ym = Yp + sin(psi) * Xc + cos(theta) * Yc 

				**********************************************	
				Xv = Xm + cos(psi) * Xc + sin(theta) * Yc 
				Yv = Ym - sin(psi) * Xc + cos(theta) * Yc 
    
        Another solution is to calculate negative of the angle

			*/

			std::vector<double> ptsx_v = std::vector<double>(num_pts); //(ptsx.size());  
			std::vector<double> ptsy_v = std::vector<double>(num_pts); // (ptsx.size());

			//caching vals
			const double cos_term = cos(-psi);
			const double sin_term = sin(-psi);
			

       //std::cout << "making car coordinate pts" << std::endl; 
			// we can use these pts for next_x and next_y also: 
      for (int i=0; i < num_pts; i++) {
				//making car's reference point according to zero
				double x_0 = ptsx[i] - px;
				double y_0 = ptsy[i] - py;	// Please note, this being set to px costed me hours of debugging
		
				ptsx_v[i] = x_0 *cos_term - y_0 * sin_term; 
				ptsy_v[i] = x_0 *sin_term + y_0 * cos_term;
			} //end forloop i transformations
      
      //std::cout << "Making eigen vector for car pts" << std::endl;
			// In order to use polyfit, we'll need to provide an Eigen::VectorXd
			Eigen::VectorXd eigenXvec = Eigen::VectorXd(num_pts) ;
			Eigen::VectorXd eigenYvec = Eigen::VectorXd(num_pts) ; 
			//Display the waypoints/reference line
      std::vector<double> next_x_vals(num_pts);
      std::vector<double> next_y_vals(num_pts);	
			for(int i=0; i < num_pts; i++) {
				eigenXvec(i) = ptsx_v[i];
				eigenYvec(i) = ptsy_v[i];
        next_x_vals[i] = ptsx_v[i];
        next_y_vals[i] = ptsy_v[i];
			} // end forloop Eigen transform	
      //std::cout << "polyfit stage" << std::endl;
			// polyfit stage		   	
			Eigen::VectorXd coeffs = polyfit(eigenXvec, eigenYvec,3);
			
      //std::cout << "polyeval stage" << std::endl;
			// getting init cte
			const double cte = polyeval(coeffs, 0) ;

			/* getting init epsi			
				The MPC model uses f0 and psi_desired  like this:
			// coeffs[3]*x0*x0*x0 + coeffs[2]*x0*x0 + coeffs[1]*x0 + coeffs[0];
			// CppAD:: atan(3*coeffs[3] * x0 *x0 + 2*coeffs[2]*x + coeffs[1]); 
		
			coeffs[1] as input to atan() and negated due to diff in sim vs MPC 
			because the x term is zero in car's ref frame			
			*/ 
      //std::cout << "getting epsi" << std::endl;      
			const double epsi = -atan(coeffs[1]);
			
			// Initializing state vector
			Eigen::VectorXd state(6); // 4 vars, 2 actuators

      /* The car is woefully unprepared to intelligently navigate without
          having latency factored into its design. Previously at early stages,
          the car would overshoot trying to follow the yellow line then 
          whip back overcorrecting and jump off the track
        **********************************************
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
        */
        double latency = 0.10;
        double v_corr = v_mps + acc*latency;
        double x_corr = v_mps * latency;
        double psi_corr = (v_mps/2.67) *-delta*latency; // delta from simulator flipped signs
        double cte_corr = cte + v_mps*sin(epsi)*latency;
        double epsi_corr = epsi * (v_mps/2.67)*-delta*latency;
      //std::cout << "assigning state vector " << v << "  " << cte << "  " << epsi << std::endl;      
			// building state, x,y,psi = 0 in car's coordinate system
			state << x_corr , 0.0 , psi_corr , v_corr , cte_corr , epsi_corr;				

		
      std::vector<double> mpc_solved = mpc.Solve(state, coeffs);	
       

       std::cout << "post mpc.Solve method" << std::endl;   	
			double steer_value;
      double throttle_value;

      std::cout << "assigning value to steer value" << std::endl; 
			steer_value = -mpc_solved[0]/ deg2rad(25);
      std::cout << "assigning value to throttle value" << std::endl;						
			
        // the acceleration is always 
      throttle_value = mpc_solved[1];
      std::cout << "Steering angle set" << steer_value << std::endl;      						
      std::cout << "throttle value set" << throttle_value << std::endl;

      /* 
				what should the MPC give back that gives some value to modifying 
				steering and throttle?		
				for sure steering_angle can use psi or delta coeff and normalize to -1,1
				
				and as for throttle? v in future state or acceleration normalized?	
			*/


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          
		      msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          std::vector<double> mpc_x_vals(15);
          std::vector<double> mpc_y_vals(15);

			for(int k = 0; k<15; k++) {
				mpc_x_vals[k] = mpc_solved[2 + k];
				mpc_y_vals[k] = mpc_solved[17 + k];
			} // end for loop k
			

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] =  next_x_vals; //ptsx_v;
          msgJson["next_y"] =  next_y_vals; //ptsy_v;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
