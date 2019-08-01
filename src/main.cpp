#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "math.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::atan2;
using std::sin;
using std::cos;
using std::pow;
using std::sqrt;
using std::exp;
using std::cout;
using std::endl;

struct Vehicle{
  int ID;
  bool parallel;
  Vehicle( int i, bool p){
    ID = i;
    parallel = p;
  }
};
//////////////////////////////////////////////////////////////////////////////////////////
// my cost function
// lane_info [speed, distance]
double calculate_cost(vector<vector<double>> lane_info, double vehicle_speed){

  double final_cost = 0; 
  for(int i = 0; i < lane_info.size(); i++){
    double this_cost;
    double spd_diff = vehicle_speed - lane_info[i][0];
    if(lane_info[i][1] > 5){  // the vehicle is ahead of us
      if(spd_diff < 0){  // if we are slower than the vehicle ahead
        this_cost = 0;
      }else{
        this_cost = 1 - exp(-1*(spd_diff)/lane_info[i][1]);  // lane_info[i][1] > 0
      }
    }else if(lane_info[i][1] < -8){ // the vehicle is behind of us
      if(spd_diff > 0){  // if we are faster than the vehicle behind
        this_cost = 0;
      }else{
        this_cost = 1 - exp(-1*(spd_diff)/lane_info[i][1]);  // lane_info[i][1] < 0
      }
    }else{  // the vehicle is parallel to us
      return 1; 
    }
    if(this_cost > final_cost){
      final_cost = this_cost;
    }
  }
  return final_cost;
}

// evaluate the cost of five state 1.LCL, 2.PLCL, 3.keep, 4.PLCR, 5.LCR
// state is only 3 lanes here
int evaluate_action(vector<vector<double>> left_lane_info, vector<vector<double>> right_lane_info,
                    vector<vector<double>> this_lane_info, double vehicle_speed){
  double min_cost = 1;
  int target_state = 1;
  for(int i = 0; i < 3; i++){
    double this_cost = 0;
    if(i < 0){
      continue;
    }else if(i > 2){
      continue;
    }else{
      if(i == state){
        this_cost = calculate_cost(this_lane_info, vehicle_speed) - 0.05;
        // cout << "Cost for this lane:" << this_cost << endl;
      }else if(i == state - 1){
        this_cost = calculate_cost(left_lane_info, vehicle_speed);
        // cout << "Cost for left lane:" << this_cost << endl;
      }else{
        this_cost = calculate_cost(right_lane_info, vehicle_speed);
        // cout << "Cost for right lane:" << this_cost << endl;
      }
    }
    if(this_cost < min_cost){
      min_cost = this_cost;
      target_state = i;
    }
  }
  return target_state;
}

void execuate_action(int target_state, int &lane){
  if(target_state == lane - 1){
    lane--;
  }else if(target_state == lane + 1){
    lane++;
  }
}

void checkParallel(int vehicleID, double distance, vector<int> thisLane){
  for(int i=0; i < thisLane.size(); i++){
    continue;
  }
}

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
  //////////////////////////// my variables
  int lane = 1;  // lane number
  double ref_vel = 0.0;  // reference velocity in mph, start from 0
  int state = 3;  // 1.LCL, 2.PLCL, 3.keep, 4.PLCR, 5.LCR
  double lane_width = 4.0;
  // a vector to store the left parallel vehicle, due to the sensor fusion unable to detect close vehicles
  vector<vector<int>> lane_parallel {{}, {}, {}};
  // vector<Vehicle> lane1_parallel;
  // vector<Vehicle> lane2_parallel;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &ref_vel, &lane]
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

          json msgJson;


          //define the actual (x, y) points on map
          vector<double> next_x_vals;
          vector<double> next_y_vals;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_size = previous_path_x.size();
          // Sensor Fusion part
          if(prev_size > 0){
            car_s = end_path_s;
          }

          bool too_close = false;
          // traffic info based on sensor fusion
          vector<vector<vector<double>>> lane_info {{}, {}, {}};
          //vector<vector<double>> this_lane_info;
          //vector<vector<double>> right_lane_info;

          // find ref_v to use
          for(int i=0; i < sensor_fusion.size(); i++){
            // cat is in my lane
            double d = sensor_fusion[i][6];   // fernet d value
            double vx = sensor_fusion[i][3];  // x, y velocity of the vehicle
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));  // over all velocity 
            double check_car_s = sensor_fusion[i][5];  // the vehicle's fernet s value
            double distance = check_car_s - car_s;
            check_car_s += ((double)prev_size * 0.02 * check_speed);
            // check the vehicle ahead
            if(d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)){
              // check_s values greater than mine and s gap (30 m)
              // if the vehicle is in front and distance less than 30m
              if((distance > -10) && (distance < 30)){
                //ref_vel = 29.5; // mph
                too_close = true;
                this_lane_info.push_back({check_speed, distance});
              }
            }

            if(d < lane_width && d >= 0){ // cehck lane 0
              if((distance > -10) && ((distance) < 30)){
                lane_info[0].push_back({check_speed, distance});
              }
            }else if(d < lane_width * 2 && d >= lane_width){ // check the lane 1
              if((distance > -10) && ((distance) < 30)){
                lane_info[1].push_back({check_speed, distance});
              }
            }else if(d <= lane_width * 3 && d >= lane_width * 2){ // check lane 2
              if((distance > -10) && ((distance) < 30)){
                lane_info[2].push_back({check_speed, distance});
              }
            }
          }
          
          // after sensor fusion 
          int target_state = evaluate_action(lane, left_lane_info, right_lane_info, this_lane_info, ref_vel);
          execuate_action(target_state, lane);

          /**
          if(too_close){
            //ref_vel -= 0.224;  // acceleration will be 5 m/s^2 
          }else if(ref_vel < 49.5){
            ref_vel += 0.224;
          }
          */
          if(ref_vel < 49.5){
            ref_vel += 0.224;
          }
          
          // Lane Following part
          // a lsit of spaced x,y points for spline
          vector<double> ptsx;
          vector<double> ptsy;
          // reference state of x, y, raw
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
           
          if(prev_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }else{
            // two of the previous points
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            // store them into the vector
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // create 3 points with 30m between two on Frenet corrdinate
          int dist = 30;  // the distance for spline anker points
          // getXY: get the x, y coordinates of map based on vehicle s, d info
          vector<double> next_wp0 = getXY(car_s + dist, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 2*dist, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 3*dist, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);


          for(int i =0; i < ptsx.size(); i++){

            //shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0.0-ref_yaw) - shift_y * sin(0.0-ref_yaw));
            ptsy[i] = (shift_x * sin(0.0-ref_yaw) + shift_y * cos(0.0-ref_yaw));
          }


          // a spline object
          tk::spline s;

          // set (x, y) points to spline
          s.set_points(ptsx, ptsy);

          // push the leftovers to the next path
          for(int i=0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // make the vehicle to go at the desired speed
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

          double x_add_on = 0;
          

          // fill up the rest of out path planner with previous points, so the output is always 50 points
          for(int i = 1; i <= 50 - previous_path_x.size(); i++){
            double N = (target_dist / (0.02 * ref_vel / 2.24));
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

            x_add_on = x_point;


            // transfer the points back to the map coordinates
            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

          // END
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////          
          
          
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