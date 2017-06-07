#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "Poly.h"
#include "json.hpp"
#include "matplotlibcpp.h"

// for convenience
using json = nlohmann::json;

namespace plt = matplotlibcpp;

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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  Poly poly;

  h.onMessage([&mpc, &poly](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::vector<double> ptsx = j[1]["ptsx"];
          std::vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];


          vector<double> ptsx_v, ptsy_v;
          // Convert desired trajectory waypoints to vehicle coordinate system
          for (unsigned int i = 0; i < ptsx.size(); i++) {

            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            double mag = sqrt(dx * dx  + dy * dy);
            double theta = atan2(dy, dx);

            ptsx_v.push_back(mag * cos(psi - theta));
            ptsy_v.push_back(-mag * sin(psi - theta));
            
          }

          // Polynomial fitting
          Eigen::Map<Eigen::VectorXd> x_vals(&ptsx_v[0], ptsx_v.size());
          Eigen::Map<Eigen::VectorXd> y_vals(&ptsy_v[0], ptsy_v.size());
          poly.Fit(x_vals, y_vals, 3);


          // Uncomment the following for debug plotting
          // std::vector<double> vx_vector;
          // std::vector<double> vy_vector;

          // {

          //   double a = 10;
          //   // Origin end of vector
          //   vx_vector.push_back(px);
          //   vy_vector.push_back(py);
          //   // Termination end of vector
          //   vx_vector.push_back(px + a * cos(psi));
          //   vy_vector.push_back(py + a * sin(psi));
            
          // };
          

          // plt::subplot(1, 2, 1);
          // plt::title("Map Coordinate System");
          // plt::plot(ptsx, ptsy, "o");
          // plt::plot(vx_vector, vy_vector);
          // plt::plot({vx_vector[0]}, {vy_vector[0]}, "*");

          // plt::subplot(1, 2, 2);
          // plt::title("Vehicle Coordinate System");
          // plt::plot(ptsx_v, ptsy_v, "o", x_poly, y_poly);

          // plt::show();

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          std::vector<double> mpc_x_vals;
          std::vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          // // Polynomial evaluation
          // std::vector<double> x_poly;
          // std::vector<double> y_poly;

          // for (double x = 0; x <= 50; x += 1.0) {

          //   auto v = poly.Eval(x);
          //   x_poly.push_back(x);
          //   y_poly.push_back(v);

          // }


          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          // vector<double> next_x_vals;
          // vector<double> next_y_vals;
          
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          // msgJson["next_x"] = next_x_vals;
          // msgJson["next_y"] = next_y_vals;
          msgJson["next_x"] = ptsx_v;
          msgJson["next_y"] = ptsy_v;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
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
