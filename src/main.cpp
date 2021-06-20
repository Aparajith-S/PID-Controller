#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <fstream>
#include "json.hpp"
#include"twiddle.h"
#include "PID.h"
// for convenience
using nlohmann::json;
using std::string;
using twiddle::Twiddle;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws)
  {
    // reset
    std::string msg("42[\"reset\", {}]");
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  }

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
  Twiddle Twu_long;
  Twiddle Twu_lat;
  PID lateral;
  PID longitudinal;
  int count=0;
  double desiredSpeed=1.0;
  /**
   * TODO: Initialize the pid variable.
   */
  lateral.Init(0.135, 0.0008, 1.0);
  longitudinal.Init(0.6, 0, 1);
  Twu_long.init({0.6, 0, 1});
  Twu_lat.init({0.135, 0.004, 4.0});
  std::ofstream fout("graph2.csv",std::ios::out);
  fout<<"steer_angle,cte_steer,steer_control,desired_velocity,actual_velocity,throttle_control"<<std::endl;
  h.onMessage([&fout,&count,&desiredSpeed,&longitudinal,&lateral,&Twu_long,&Twu_lat](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value,throttle;
          
          if(fabs(cte)>=4.5)
          {
            reset_simulator(ws);
            Twu_long.cyclic(longitudinal);
            Twu_lat.cyclic(lateral);
          }

          // lateral control
          lateral.UpdateError(cte);
          steer_value=lateral.Control();
          constexpr double kMaxSpeed =100.0;
          constexpr double kMaxAngle = 25.0;
          desiredSpeed = std::max(0.0, kMaxSpeed * ( 1.0 - fabs((angle/kMaxAngle)*cte) / 4.0));
          desiredSpeed = std::min(100.0, desiredSpeed);
          double speederror = speed - desiredSpeed;
          // longitudinal control
          longitudinal.UpdateError(speederror);
          throttle=longitudinal.Control();
          // DEBUG
          /*std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                   << "throttl : "<<throttle << std::endl;*/
          json msgJson;
          throttle=saturate(0.7 +throttle,-1.0,0.3);
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          fout<<angle<<","<<cte<<","<<steer_value<<","<<desiredSpeed<<","<<speed<<","<<throttle<<std::endl;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
         // std::cout << msg << std::endl;
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
  fout.close();
}
