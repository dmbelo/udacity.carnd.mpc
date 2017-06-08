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

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {

  double result = 0.0;
  for (unsigned int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }

  return result;

}

double polydiff(Eigen::VectorXd coeffs, double x) {
    
    double result = 0.0;
    for (unsigned int i = 1; i < coeffs.size(); i++) {
        result += i * coeffs[i] * pow(x, i - 1);
    }

    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {

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

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {

  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";

}

int main() {

  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {

      std::string s = hasData(sdata);
      if (s != "") {

        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::vector<double> x_waypts = j[1]["ptsx"];
          std::vector<double> y_waypts = j[1]["ptsy"];
          double x_car = j[1]["x"];
          double y_car = j[1]["y"];
          double psi_car = j[1]["psi"];
          double v_car = j[1]["speed"];

          std::vector<double> xv_waypts, yv_waypts;
          // Convert desired trajectory waypoints to vehicle coordinate system
          for (unsigned int i = 0; i < x_waypts.size(); i++) {

            double dx = x_waypts[i] - x_car;
            double dy = y_waypts[i] - y_car;
            double mag = sqrt(dx * dx  + dy * dy);
            double theta = atan2(dy, dx);

            xv_waypts.push_back(mag * cos(psi_car - theta));
            yv_waypts.push_back(-mag * sin(psi_car - theta));
            
          }

          // Polynomial fitting
          Eigen::VectorXd coeffs;

          {

            Eigen::Map<Eigen::VectorXd> x(&xv_waypts[0], xv_waypts.size());
            Eigen::Map<Eigen::VectorXd> y(&yv_waypts[0], yv_waypts.size());
            // poly.Fit(x, y, 3);
            coeffs = polyfit(x, y, 3);
            
          }

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // double cte_car = poly.Eval(x_car) - y_car;
          // double err_psi_car = psi_car - atan(poly.Diff(x_car));
          double cte_car = polyeval(coeffs, x_car) - y_car;
          double err_psi_car = psi_car - atan(polydiff(coeffs, x_car));

          Eigen::VectorXd states(6);

          // Vehicle states in the vehicle coordinate system
          states << 0, 0, 0, v_car, cte_car, err_psi_car;

          auto vars = mpc.Solve(states, coeffs);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          double a_steer = vars[6]/deg2rad(25);
          double r_throttle = vars[7];
          // double a_steer = 0.0;
          // double r_throttle = 0.0;

          std::cout << a_steer << ", " << r_throttle << std::endl << std::endl;

          msgJson["steering_angle"] = -a_steer;
          msgJson["throttle"] = r_throttle;

          // //Display the MPC predicted trajectory 
          // std::vector<double> mpc_x_vals;
          // std::vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc.xv_opt_traj;
          msgJson["mpc_y"] = mpc.yv_opt_traj;

          //Display the waypoints/reference line

          // Polynomial evaluation
          std::vector<double> xv_ref_traj;
          std::vector<double> yv_ref_traj;

          for (double x = 0; x <= 50; x += 5.0) {

            // double v = poly.Eval(x);
            double v = polyeval(coeffs, x);
            xv_ref_traj.push_back(x);
            yv_ref_traj.push_back(v);

          }
        
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = xv_ref_traj;
          msgJson["next_y"] = yv_ref_traj;

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
          // std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
  // program doesn't compile :-(
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
