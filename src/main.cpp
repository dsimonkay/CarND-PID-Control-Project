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

  // initializing the P, I and D coefficients
  double Kp = 0.03;
  double Ki = 0.001;
  double Kd = 1.1;

  // the PID variable will be initialized implicitly by the twiddler
  // (regardless of whether we'll do the twiddle thing or not).
  PID pid;
  Twiddle twiddle(do_twiddling, Kp, Ki, Kd);
  twiddle.start(pid);

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

          // sorry, guys
          // double speed = std::stod(j[1]["speed"].get<std::string>());
          // double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          // classic PID processing steps
          pid.updateError(cte);
          double steering = pid.calculateSteering(cte);
          double throttle = pid.calculateThrottle(cte);

          // we have things to do in case twiddling is active
          if ( twiddle.isActive() ) {

            int twiddle_status = twiddle.check(pid, cte);
            if ( twiddle_status == Twiddle::FINISHED ) {

              // goodbye, Mr. Anderson
              exit(0);

            } else if ( twiddle_status == Twiddle::RESTART_LOOP ) {

                // restarting the simulator and the PID controller as well
                resetSimulator(ws);
                twiddle.start(pid);
                return;
            }

          } else {
            // basic debug output
            std::cout << "CTE, steering, throttle:   " << cte << ",  " << steering << ",  " << throttle << std::endl;
          }

          json msgJson;
          msgJson["steering_angle"] = steering;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
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
