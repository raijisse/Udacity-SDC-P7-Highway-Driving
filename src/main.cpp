#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;


double ref_vel = 0.0;
int lane = 1;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Initialize lane, 0 would be left, 1 middle and 2 right
  
  // Set speed instruction
  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Get the size of the previous path poitns still valid
          int prev_size = previous_path_x.size();

          if(prev_size>0){
            car_s = end_path_s;
          }

          // Initialize a value that will keep track of our car getting too close of other cars
          bool too_close = false;
          bool on_the_left = false;
          bool on_the_right = false;

          // For each car detected by the sensors we are going to see if
          // they are a potential infringement to our progress
          for (int i=0; i<sensor_fusion.size(); i++){

            int safety_distance = 30;
            // d coordinates
            float d = sensor_fusion[i][6];
            
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)prev_size*.02*check_speed);
            
            // Check where other cars are:
            // 1) Check if a car is in my lane
            if(d < (2+4*lane+2) && d > (2+4*lane-2)){ 
              // If it is in my lane, check the distance between the ego
              // vehicle and the other one. 
              if((check_car_s > car_s) && ((check_car_s - car_s) < safety_distance)){
                too_close = true;
                }
              }
            
            // 2) Check if a car is on the lane to my left
            else if(d < (4*lane) && d > (4*lane-4)){
              // Check if there is enough space to pass the vehicle in front of me
              if((check_car_s > (car_s-safety_distance)) && ((check_car_s < (car_s+safety_distance)))){
                on_the_left = true;
                }              
            }

            // 3) Check if a car is on the lane to my right
            else if((d > (4*lane+4)) && (d < (4*lane + 8))){
              // Check if there is enough space to pass the vehicle in front of me
              if((check_car_s > (car_s-safety_distance)) && ((check_car_s < (car_s+safety_distance)))){
                on_the_right = true;
                }
            
            }
          }
          
          // If there is a car ahead
          if (too_close == true){
            // and that the left lane is empty, we can go the left lane
            if ((on_the_left == false) && (lane >= 1)){
              lane -= 1;
            }
            // otherwise we try to go to the right lane
            else if ((on_the_right == false) && (lane <2)){
              lane += 1;
            }
            // If no passing is possible, we reduce speed
            else {
              ref_vel -= 0.224;
            }
          }
          // Finally, if there is no car ahead, we just check that the current speed is close
          // to the target speed, otherwise we accelerate
          else {
            if((ref_vel < 49)){
              ref_vel += 0.224;
            }
          }


          // Initialize the x and y waypoints
          vector<double> xpts;
          vector<double> ypts;

          // Keep track of x, y and yaw :
          // Starting point or previous path point
          double x_ref = car_x;
          double y_ref = car_y;
          double yaw_ref = deg2rad(car_yaw);

          // If we do not have waypoints, create two points from the starting point

          if(prev_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            // The first waypoint is an estimated one based on current yaw rate (tangent trajectory)
            xpts.push_back(prev_car_x);
            // The second one is the starting (current at t=0) position
            // Hence we populate waypoints for the first iteration
            xpts.push_back(car_x);
            // Likewise for y
            ypts.push_back(prev_car_y);
            ypts.push_back(car_y);


          }
          else {
            x_ref = previous_path_x[prev_size-1];
            y_ref = previous_path_y[prev_size-1];

            double x_ref_prev = previous_path_x[prev_size-2];
            double y_ref_prev = previous_path_y[prev_size-2];
            yaw_ref = atan2(y_ref-y_ref_prev, x_ref-x_ref_prev);

            xpts.push_back(x_ref_prev);
            xpts.push_back(x_ref);

            ypts.push_back(y_ref_prev);
            ypts.push_back(y_ref);
          }

          // In Frenet coordinates, add 30m evenly spaced waypoints
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          xpts.push_back(next_wp0[0]);
          xpts.push_back(next_wp1[0]);
          xpts.push_back(next_wp2[0]);

          ypts.push_back(next_wp0[1]);
          ypts.push_back(next_wp1[1]);
          ypts.push_back(next_wp2[1]);

          for (int i = 0; i < xpts.size(); ++i) {
            
            // Shift in rotation to make the maths easier
            double x_shift = xpts[i]-x_ref;
            double y_shift = ypts[i]-y_ref;

            xpts[i] = (x_shift * cos(0-yaw_ref) - y_shift * sin(0-yaw_ref));
            ypts[i] = (x_shift * sin(0-yaw_ref) + y_shift * cos(0-yaw_ref));
          }

          tk::spline s;

          // Set xpts and ypts to the spline
          s.set_points(xpts, ypts);

          // Initialize the next x and y waypoints
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Append previous paths points
          for(int i=0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0; //m
          double target_y = s(target_x); //m
          double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

          double x_add_on = 0;

          // Fill up the rest of our waypaths points
          for(int i = 1; i <= 50 - previous_path_x.size(); i++){

            //Compute the number of points we have to add
            double N = (target_dist / (0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double ref_x = x_point;
            double ref_y = y_point;

            //rotate back to normal
            x_point = (ref_x *cos(yaw_ref) - ref_y*sin(yaw_ref));
            y_point = (ref_x *sin(yaw_ref) + ref_y*cos(yaw_ref));
            
            x_point += x_ref;
            y_point += y_ref;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

          json msgJson;
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
}