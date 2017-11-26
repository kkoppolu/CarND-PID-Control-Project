#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "PID.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
const int NumSteps = 3200;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void doReset(uWS::WebSocket<uWS::SERVER>& ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

int main() {
  uWS::Hub h;

  PID pid;
  PID speedPid;
  // TODO: Initialize the pid variable.
  // 0.09, 0, 0.5
  double p[] = {0.133155, 0.0000583929, 1.23517};
  pid.Init(p[0], p[1], p[2]);
  double dp[] = {0.01, 0.0001, 0.1};
  int numSteps = 0;
  bool doTwiddle = false;
  int pIdx = 0;      // parameter to tune
  int direction[] = {1, 0, 0};  // up (1)/down(-2)/restore(1)
  double best_error = std::numeric_limits<double>::max();
  double error = 0;
  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          if (numSteps == 0) {
            p[0] += direction[0] * dp[0];
            p[1] += direction[1] * dp[1];
            p[2] += direction[2] * dp[2];
            pid.Init(p[0], p[1], p[2]);
          }
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          pid.UpdateError(cte);
          // double speed = std::stod(j[1]["speed"].get<std::string>());
          // double angle =
          // std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0;
          double totalError = pid.TotalError();
          if (numSteps >= NumSteps/2) {
            error += cte * cte;
          }
          
          if (totalError < -1) {
            totalError = -1;
          } else if (totalError > 1) {
            totalError = 1;
          }
          steer_value = totalError;

          // DEBUG
          // std::cout << "[" << numSteps << "]CTE: " << cte << " Steering Value: " << steer_value
          //          << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          ++numSteps;
          doTwiddle = doTwiddle && (dp[0] + dp[1] + dp[2] > 0.001);
          if (numSteps >= NumSteps && doTwiddle) {
            std::cout << "Twiddling" << std::endl;
            error  = error * 2.0/ NumSteps;
            if (std::abs(error) < std::abs(best_error)) {
              best_error = error;
              dp[pIdx] *= 1.1;
              memset(direction, 0, sizeof(direction[0]) * 3); 
              ++pIdx; // move onto the next param
              pIdx = pIdx % 3; // circulate among the 3 params
              direction[pIdx] = 1; // start moving the next param up
              std::cout << "Best Parameters: (" << p[0] << ", " << p[1] << ", " << p[2] << ")"
              << std::endl;
              std::cout << "Best error: " << best_error << std::endl;
            } else if (direction[pIdx] == 1) {
              direction[pIdx] = -2; // move down
            } else {
              p[pIdx] += dp[pIdx]; // restore
              dp[pIdx] *= 0.9; // decrease dp
              memset(direction, 0, sizeof(direction[0]) * 3); 
              ++pIdx; // move onto the next param
              pIdx = pIdx % 3; // circulate among the 3 params
              direction[pIdx] = 1; // start moving the next param up   
            } 

            error = 0;
            numSteps = 0;
            doReset(ws);
          }
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
