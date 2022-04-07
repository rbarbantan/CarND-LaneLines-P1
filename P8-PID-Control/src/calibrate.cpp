#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <cmath>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"

// for convenience
using nlohmann::json;
using std::string;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const int MAX_FRAMES = 1200;
const double target_speed = 30.0;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID lat_pid;
  lat_pid.Init(0.0, 0.0, 0.0);
  
  PID long_pid;
  long_pid.Init(1.0, 0.0, 0.0);

  int frame = 0;
  double total_cte = 0;

  Twiddle twiddle;

  h.onMessage([&lat_pid, &long_pid, &frame, &total_cte, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          //double angle = std::stod(j[1]["steering_angle"].get<string>());
          total_cte += std::abs(cte);
              
          lat_pid.UpdateError(cte);
          double steer_value = lat_pid.TotalError();

          long_pid.UpdateError(speed-target_speed);
          double throttle_value = long_pid.TotalError();
          //std::cout << frame << " speed: " << speed <<  " cte: " << cte << std::endl;

          if (frame >= MAX_FRAMES || (frame > 10 && std::abs(cte) > 6) || (frame > 100 && speed < 1)) {
            if (!twiddle.IsDone()) {
              //double err = lat_pid.TotalError();
              double err = MAX_FRAMES - frame + total_cte/frame;
              std::cout << frame <<", " << err << std::endl;
              twiddle.Update(err);
              vector<double> params = twiddle.GetCurrentParams();
              std::cout << "current search: " << params[0] << ", " << params[1] << ", " << params[2] << std::endl;
              //vector<double> bparams = twiddle.GetBestParams();
              //std::cout << "best search: " << bparams[0] << ", " << bparams[1] << ", " << bparams[2] << std::endl;
              vector<double> dp = twiddle.GetDP();
              std::cout << "current dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
              lat_pid.Init(params[0], params[1], params[2]);
              //reset sim
              std::string msg = "42[\"reset\"]";
              std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              frame = 0;
              total_cte = 0;
              return;
            } else {
              vector<double> result = twiddle.GetBestParams();
              std::cout << "stopped search: " << result[0] << ", " << result[1] << ", " << result[2] << std::endl;
            }
          }

          frame += 1;
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Speed Value: " << speed << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    //std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}