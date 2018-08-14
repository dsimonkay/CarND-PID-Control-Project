#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <uWS/uWS.h>

/*
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 */
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


/*
 * Sending a RESET signal to the simulator
 */
void resetSimulator(uWS::WebSocket<uWS::SERVER> &ws) {
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}


/*
 * Simple Is-Numeric function in C++.
 * Original source: https://gautiertalkstechnology.wordpress.com/2015/04/20/cplusplus-isnumeric-function/
 */
bool is_digit(const char c) {
  return std::isdigit(c) || c == '.';
}

bool is_numeric(const std::string &s) {
  return std::all_of(s.begin(), s.end(), is_digit);
}


#endif /* HELPER_FUNCTIONS_H_ */
