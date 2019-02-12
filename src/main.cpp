#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

//pid controller 
#include "pid.h"
#include "spline.h"

// for convenience
using nlohmann::json;
//using std::string;
//using std::vector;
using namespace std;

void compute_spline_based_trajectory(const int &target_lane, const double &car_s, const double &car_x,
                       const double &car_y, const double &car_yaw, const vector<double> &previous_path_x,
                       const vector<double> &previous_path_y, const vector<double> &map_waypoints_x,
                       const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s,
                       const double &ref_velocity, vector<double> &next_x_vals, vector<double> &next_y_vals) {
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  const int prev_size = previous_path_x.size();


  if ( prev_size < 2 ) {

      const double prev_car_x = car_x - cos(car_yaw);
      const double prev_car_y = car_y - sin(car_yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);
  } else {

      ref_x = previous_path_x[prev_size - 1];
      ref_y = previous_path_y[prev_size - 1];

      const double ref_x_prev = previous_path_x[prev_size - 2];
      const double ref_y_prev = previous_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
  }

  const double step_size = 0.6 * ref_velocity;
  vector<double> next_wp0 = getXY(car_s + step_size, 2 + 4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + step_size * 2.0, 2 + 4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + step_size * 3.0, 2 + 4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);


  for ( int i = 0; i < ptsx.size(); i++ ) {
    const double shift_x = ptsx[i] - ref_x;
    const double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }


  tk::spline s;
  s.set_points(ptsx, ptsy);

  for ( int i = 0; i < prev_size; i++ ) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }


  const double target_x = 30.0;
  const double target_y = s(target_x);
  const double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for( int i = 1; i < 50 - prev_size; i++ ) {
    const double N = target_dist / (0.02 * ref_velocity / 2.24);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    const double x_ref = x_point;
    const double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
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

  // set up the driving speed limits
  constexpr double MAX_VELOCITY = 49.5;
  constexpr double VELOCITY_DELTA = 0.6;
  constexpr double MIN_FOLLOWING_SPEED_FRACTION = 0.9;
  constexpr double LANE_CHANGE_SPEED_SPACE_FRACTION = 0.3;
  //assumption of 3 lanes
  constexpr int NUM_LANES = 3;

  const double linux_parameters[] = {0.015,0,0.2};

  // initialize PID controller and tracking variable
  PID pid;
  pid.init(linux_parameters);
  bool started_lane_change = false;
  int target_lane = 1;
  double ref_velocity = 0; 

  h.onMessage([&VELOCITY_DELTA, &MAX_VELOCITY, &target_lane, &started_lane_change, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_velocity, &pid]
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

	  //determine secondary state of car

	  const int car_lane = car_d / 4.0;
	  double follow_distance = max(10.0, car_speed);
	  const double lane_change_gap = max(10.0, car_speed * LANE_CHANGE_SPEED_SPACE_FRACTION);
	  const bool lane_change_occuring = ((int)round(car_d) % 4) < 1;

	  // detect lane changes in progress
          if (started_lane_change) {
            if (lane_change_occuring) {
              started_lane_change = false;
            }
          }

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
	
          const int prev_size = previous_path_x.size();
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // determine if there are cars to the left or right, and how close they are
          double closest_car = std::numeric_limits<double>::max();
          bool car_left = false;
          bool car_right = false;
          double car_left_gap = std::numeric_limits<double>::max();
          double car_right_gap = std::numeric_limits<double>::max();
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
              const float d = sensor_fusion[i][6];
              const int other_lane = d / 4.0;
              const double vx = sensor_fusion[i][3];
              const double vy = sensor_fusion[i][4];
              const double other_speed = sqrt(vx*vx+vy*vy);
              double other_s = sensor_fusion[i][5];

              other_s += ((double) prev_size * 0.02 * other_speed);

              const double gap = other_s - car_s;

              if (other_lane == car_lane) {
                if (gap > 0 && closest_car > gap) {
                  closest_car = gap;
                }
              } else if (!car_left && other_lane == (car_lane - 1)) {
                // lane left of car
                if (-lane_change_gap < gap && gap < lane_change_gap) {
                  car_left = true;
                }
                if (0.0 < gap && gap < car_left_gap) {
                  car_left_gap = gap;
                }

              } else if (other_lane == (car_lane + 1)) {
                // lane right of car
                if (-lane_change_gap < gap && gap < lane_change_gap) {
                  car_right = true;
                }
                if (0.0 < gap && gap < car_right_gap) {
                  car_right_gap = gap;
                }
              }
          }


          // should the car attempt to change lanes?
          if (!started_lane_change && !lane_change_occuring && closest_car < MAX_VELOCITY && ref_velocity < MAX_VELOCITY * MIN_FOLLOWING_SPEED_FRACTION) {
            if (!car_left && car_left_gap > closest_car && ( // no car on left and left lane is more open ahead than center
                  (car_lane == NUM_LANES - 1 || (car_lane > 0 && car_right)) || // in farthest right lane or in center with right blocked
                  (car_lane > 0 && !car_right && car_left_gap >= car_right_gap))) { // in center lane with right unblocked and left more open ahead than right
              // shift left
              target_lane--;
              started_lane_change = true;
            } else if (car_lane < NUM_LANES - 1 && car_right_gap > closest_car && !car_right) { // not in right lane, right lane is open more ahead than center, right lane unblocked
              // shift right
              target_lane++;
              started_lane_change = true;
            }
          }

	
          // compute acceleration, smoothed by a PID controller and bounded max maximum acceleration for comfort
          const double acceleration = pid.compute_control_value((follow_distance - closest_car));
          const double trimmed_acceleration = min(max(-3.0 * VELOCITY_DELTA, acceleration), VELOCITY_DELTA);

          // compute new reference velocity bounded by 2.0 and speed limit
          ref_velocity = max(min(MAX_VELOCITY, ref_velocity + trimmed_acceleration), 5.0);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // compute trajectory based on target lane and velocity
          compute_spline_based_trajectory(target_lane, car_s, car_x, car_y, car_yaw, previous_path_x,
                     previous_path_y, map_waypoints_x,
                     map_waypoints_y, map_waypoints_s,
                     ref_velocity, next_x_vals, next_y_vals);

          json msgJson;


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


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
