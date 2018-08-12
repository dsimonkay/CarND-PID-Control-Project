#include <uWS/uWS.h>
#include <algorithm>
#include <iostream>
#include "json.hpp"
#include "PID.h"
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

int step_count = 0;

int main(int argc, char* argv[]) {

  uWS::Hub h;

  PID pid;

  double debug = argc > 1 && (std::string(argv[1]) == "-t" || std::string(argv[1]) == "--twiddle");

  // TODO: Initialize the pid variable.
  // pid.Init(0.2, 0.0001, 4.0);
  // pid.Init(0.08, 0.000005, 2.8); // so-so
  // pid.Init(0.08, 0.000005, 2.8); // so-so
  double Kp = 0.05;
  double Ki = 0.00001;
  double Kd = 1.0;
  pid.Init(Kp, Ki, Kd);

  // writing the CTE, steering and throttle values in a CSV file in debug mode
  std::ofstream pid_values;
  if ( debug ) {

      // assembling the filename. it has the following structure: PID_<date>_<time>_<Kp>_<Ki>_<Kd>.csv
      std::stringstream filename;
      time_t curr_time;
      char datetime_buffer[20];
      time(&curr_time);
      strftime(datetime_buffer, 20, "%Y%m%d_%H%M%S", localtime(&curr_time));
      filename << "PID_" << std::string(datetime_buffer) << "_" << Kp << "_" << Ki << "_" << Kd << ".csv";
      std::cout << "Filename: " << filename.str() << std::endl;

      pid_values.open(filename.str().c_str(), std::ios_base::out | std::ios_base::app);
      pid_values << "CTE,steering,throttle" << std::endl;
      // pid_values.close();
  }


  h.onMessage([&pid, &pid_values](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steering = pid.CalculateSteering(speed, angle, cte);
          // throttle = pid.CalculateThrottle(speed, angle, cte);
          throttle = 0.3;

          // DEBUG
          if ( pid_values.is_open() ) {
            pid_values << cte << "," << steering << "," << throttle << std::endl;
          }


          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steering << std::endl;
          std::cout << "step: " << ++step_count << std::endl;

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
