#include <uWS/uWS.h>
#include <algorithm>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"
#include <math.h>
#include <fstream>
#include <sstream>
#include <ctime>


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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


// Sending a RESET signal to the simulator
void resetSimulator(uWS::WebSocket<uWS::SERVER> &ws) {

  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}


int main(int argc, char* argv[]) {

  uWS::Hub h;

  bool do_twiddling = argc > 1 && (std::string(argv[1]) == "-t" || std::string(argv[1]) == "--twiddle");
  if ( do_twiddling ) {
    std::cout << "Twiddle has been activated." << std::endl;
  }

  // initialize the P, I and D coefficients
  double Kp = 0.1;
  double Ki = 0.00001;
  double Kd = 1.0;

  // the PID variable will be initialized implicitly by the twiddler (regardless of whether we'll
  // do the twiddling or not)  
  PID pid;
  Twiddle twiddle = Twiddle(do_twiddling, Kp, Ki, Kd);
  twiddle.Init(pid);

  h.onMessage([&pid, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {

        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {

          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          double steering;
          double throttle;

          std::cout << "cte: " << cte << "  step " << twiddle.step_count << std::endl;

          // classic PID processing steps
          pid.UpdateError(cte);
          steering = pid.CalculateSteering(cte);
          throttle = pid.CalculateThrottle(cte);

          // DEBUG!!!
          steering = -1.0;

          // we have other things to do in case twiddling is active
          if ( twiddle.isActive() ) {

            // breaking the curent twiddle session in case the curent CTE is simply too big
            if ( std::abs(cte) > Twiddle::CTE_LIMIT ) {

              std::cout << " ** CTE (" << cte << ") over limit; finishing twiddle cycle." << std::endl;
              twiddle.Init(pid);
              resetSimulator(ws);
              return;
            }

            unsigned int twiddle_result = twiddle.doOneStep(pid);
            if ( twiddle_result == Twiddle::FINISHED ) {

            }
          }

          json msgJson;
          msgJson["steering_angle"] = steering;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
