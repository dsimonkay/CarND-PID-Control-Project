#include <algorithm>
#include <iostream>
#include "helper_functions.h"
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"


// for convenience
using json = nlohmann::json;


int main(int argc, char* argv[]) {

  uWS::Hub h;

  // Flag for twiddle mode. Its value can be set to `true` by providing the command line parameter "-t" or "--twiddle".
  bool do_twiddling = false;

  // Initializing the P, I and D coefficients. These can be overridden by command line parameters "-Kp", "-Ki" and "-Kd"
  // Example: user@localhost:~/CarND-PID-Control-Project/build$ ./pid -Kp 0.04 -Ki 0.001 -Kd 1.4 --twiddle

  // double Kp = 0.04;
  // double Ki = 0.001;
  // double Kd = 1.4;

  double Kp = 0.05;
  double Ki = 0.001;
  double Kd = 1.2;

  // Processing command line parameters
  for( int i = 1;  i < argc;  i++ ) {

    std::string param = std::string(argv[i]);
    std::string next_param = argc > (i+1) ? std::string(argv[i+1]) : "x";
    std::string param_lower;
    std::transform(param.begin(), param.end(), std::back_inserter(param_lower), ::tolower);

    if ( is_numeric(next_param) ) {

      if ( param_lower == "-kp" ) {
        Kp = std::stod(next_param);

      } else if ( param_lower == "-ki" ) {
        Ki = std::stod(next_param);

      } else if ( param_lower == "-kd" ) {
        Kd = std::stod(next_param);
      }

    } else if ( param_lower == "-t" || param_lower == "--twiddle" ) {
      do_twiddling = true;
    }
  }

  std::cout << "Base parameters:  [Kp, Ki, Kd]: [" << Kp << ", " << Ki << ", " << Kd << "]" << std::endl;

  // The PID variable will be initialized implicitly by the twiddler
  // (regardless of whether we'll do the twiddle thing or not).
  PID pid;
  Twiddle twiddle(do_twiddling, Kp, Ki, Kd);
  twiddle.startLoop(pid);

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

          // Sorry, guys, but you're out of the game.
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
                twiddle.endLoop(pid);
                resetSimulator(ws);
                twiddle.startLoop(pid);

                return;
            }

          } else {
            // basic debug output
            std::cout << "Max.CTE so far   current CTE   steering   throttle    ---    " << pid.getMaxCTE() << "   " <<
                                           cte << "   " << steering << "   " << throttle << std::endl;
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
