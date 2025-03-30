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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Start in lane 1, which is the center lane (0 is left and 2 is right)
  int lane = 1;
  // Start at zero velocity and gradually accelerate
  double ref_vel = 0.0; // mph
  
  // Constants for speed and acceleration control
  const double MAX_SPEED = 49.5; // mph, just under 50 mph limit
  const double MAX_ACC = 0.224;  // Maximum acceleration increment (prevents jerk)
  const double SAFE_DISTANCE = 30.0; // Safe distance to keep from vehicle ahead (meters)
  
  // Lane change state
  bool changing_lanes = false;
  int target_lane = 1;
  
  // Track how long we've been stuck behind someone
  int stuck_counter = 0;
  const int PATIENCE_THRESHOLD = 100; // Number of iterations before considering riskier maneuvers
  
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_vel,&lane,
               &changing_lanes,&target_lane,&MAX_SPEED,&MAX_ACC,&SAFE_DISTANCE,&max_s,
               &stuck_counter,&PATIENCE_THRESHOLD]
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

          /**
           * The given code defines a path made up of (x,y) points that the car
           *   will visit sequentially every .02 seconds
           */

          int prev_size = previous_path_x.size();
          
          // Update car_s to the end of the previous path
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          // Handle lane change in progress
          if (changing_lanes && lane != target_lane) {
            // Check if we're close to center of lane
            if (fabs(car_d - (2 + 4*lane)) < 0.5) {
              // We've completed the lane change
              lane = target_lane;
              changing_lanes = false;
            }
          }
        
          // Flags for behavior
          bool too_close = false;
          vector<bool> lane_is_safe(3, true); // Assume all lanes are safe initially
          
          // Store closest vehicle ahead in each lane and its distance
          vector<double> closest_front(3, 999999.0);
          vector<double> closest_front_speed(3, 0);
          
          // Default safety distance (will be adjusted with speed)
          double dynamic_safe_distance = std::max(SAFE_DISTANCE, car_speed * 0.447 * 2.0);
          
          // Handle track loop
          for (int i = 0; i < sensor_fusion.size(); i++) {
            // Extract vehicle data
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            
            // Handle the case where car crosses finish line (s wraps around)
            if (check_car_s < 100 && car_s > max_s - 100) {
              check_car_s += max_s;
            } else if (car_s < 100 && check_car_s > max_s - 100) {
              check_car_s -= max_s;
            }
            
            // Project vehicle s position into the future
            check_car_s += (double)prev_size * 0.02 * check_speed;
            
            // Determine lane of other vehicle
            int check_car_lane = -1;
            if (d > 0 && d < 4) {
              check_car_lane = 0; // Left lane
            } else if (d > 4 && d < 8) {
              check_car_lane = 1; // Middle lane
            } else if (d > 8 && d < 12) {
              check_car_lane = 2; // Right lane
            }
            
            // Skip if not in a valid lane
            if (check_car_lane < 0) continue;
            
            // Calculate dynamic safety distance based on speed
            // Higher speeds require more following distance (2 seconds rule)
            dynamic_safe_distance = std::max(SAFE_DISTANCE, car_speed * 0.447 * 2.0);
            
            // Check if vehicle is ahead or behind
            if (check_car_s > car_s) {
              // Vehicle is ahead - check distance and update closest vehicle data
              double distance_front = check_car_s - car_s;
              
              if (distance_front < closest_front[check_car_lane]) {
                closest_front[check_car_lane] = distance_front;
                closest_front_speed[check_car_lane] = check_speed;
              }
              
              // Mark lane as unsafe if vehicle is too close ahead
              if (distance_front < dynamic_safe_distance) {
                lane_is_safe[check_car_lane] = false;
                
                // Flag as too close if vehicle is in our lane
                if (check_car_lane == lane) {
                  too_close = true;
                }
              }
            } else {
              // Vehicle is behind - still need to check safety for lane changes
              double distance_back = car_s - check_car_s;
              
              // If car is approaching from behind at high speed, mark lane as unsafe
              // Check if it would reach us within 3 seconds
              double time_to_collision = distance_back / (check_speed > car_speed*0.447 ? (check_speed - car_speed*0.447) : 0.001);
              if (distance_back < 15 || (distance_back < 50 && time_to_collision < 3.0)) {
                lane_is_safe[check_car_lane] = false;
              }
            }
          }
          
          // Behavior planning - decide what to do
          if (too_close) {
            // Need to slow down
            ref_vel -= MAX_ACC;
            
            // Increment stuck counter when we're behind a slower vehicle
            if (ref_vel < MAX_SPEED - 5) {
              stuck_counter++;
            }
            
            // If extremely close to the front car, apply emergency braking
            if (closest_front[lane] < 15) {
              // Apply heavier braking to avoid collision
              ref_vel -= MAX_ACC * 2;
            }
            
            // Match speed with front car if we're getting close and can't change lanes
            if (closest_front[lane] < dynamic_safe_distance && closest_front[lane] > 0) {
              // If we can't change lanes, try to match the speed of the car in front
              if ((lane == 0 && !lane_is_safe[1]) || (lane == 2 && !lane_is_safe[1]) || 
                  (lane == 1 && !lane_is_safe[0] && !lane_is_safe[2])) {
                // Get target vehicle speed and match it
                double front_car_speed = closest_front_speed[lane];
                // Convert from m/s to mph (0.447 conversion factor)
                double front_car_mph = front_car_speed * 2.24;
                // Match speed but stay slightly slower to increase gap
                if (ref_vel > front_car_mph - 1) {
                  ref_vel = std::max(front_car_mph - 1, ref_vel - MAX_ACC * 2);
                }
              }
            }
            
            // Consider changing lanes if not already changing
            if (!changing_lanes) {
              // Variables to help decide best lane
              int best_lane = lane;
              double best_cost = 999999;
              
              // If we've been patient enough, we might take a slightly riskier lane change
              // Only when we've been stuck behind slow traffic for a while
              double safety_threshold = stuck_counter > PATIENCE_THRESHOLD ? 0.8 : 1.0;
              
              // Check if adjacent lanes are safe for changing
              for (int l = 0; l < 3; l++) {
                // Skip current lane and lanes more than 1 away (can't change to lane 0 from lane 2)
                if (l == lane || abs(l - lane) > 1) continue;
                
                // Get the safety buffer we want to enforce
                double required_front_distance = dynamic_safe_distance * safety_threshold;
                
                // When we're desperate (stuck for too long), consider lane even if not perfectly safe
                bool is_lane_viable = lane_is_safe[l];
                if (!is_lane_viable && stuck_counter > PATIENCE_THRESHOLD * 2) {
                  // If we've been stuck for a very long time, check if the lane is at least somewhat viable
                  is_lane_viable = closest_front[l] > required_front_distance * 0.7;
                }
                
                // Only consider lane if it's safe
                if (is_lane_viable) {
                  // Compute cost based on closest vehicle in lane and speed difference
                  double cost = 0;
                  
                  // If there's a vehicle ahead in this lane, add cost proportional to closeness 
                  if (closest_front[l] < 999999) {
                    cost += (100.0 / closest_front[l]);
                    
                    // Add cost for slower vehicles
                    double speed_diff = car_speed*0.447 - closest_front_speed[l];
                    if (speed_diff > 0) {
                      cost += speed_diff * 10;
                    }
                  }
                  
                  // Prefer center lane slightly
                  if (l != 1) {
                    cost += 10;
                  }
                  
                  // Update best lane if this is better
                  if (cost < best_cost) {
                    best_cost = cost;
                    best_lane = l;
                  }
                }
              }
              
              // Change lane if a better lane was found
              if (best_lane != lane) {
                target_lane = best_lane;
                changing_lanes = true;
                // Reset stuck counter after successful lane change decision
                stuck_counter = 0;
              }
            }
          } else if (ref_vel < MAX_SPEED) {
            // No vehicle close ahead, accelerate if below speed limit
            ref_vel += MAX_ACC;
            
            // Reset stuck counter since we're not stuck
            stuck_counter = 0;
            
            // If not in center lane and not changing lanes, consider returning to center
            if (lane != 1 && !changing_lanes && lane_is_safe[1]) {
              // Return to center lane if it's safe and has good clearance
              if (closest_front[1] > 50) {
                target_lane = 1;
                changing_lanes = true;
              }
            }
          }
          
          // Create a list of evenly spaced waypoints 30m apart
          // Interpolate those waypoints later with spline and fill it in with more points
          vector<double> ptsx;
          vector<double> ptsy;
        
          // Reference x, y, yaw states, either will be the starting point or end point of the previous path
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
        
          // if previous size is almost empty, use the car as starting reference
          if (prev_size < 2) {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - 0.5 * cos(car_yaw);
            double prev_car_y = car_y - 0.5 * sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // Use the previous path's end point as starting reference
          else {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev , ref_x - ref_x_prev);
            // Use the two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
        
          // Target lane is where we want to be, which might be different from current lane during transition
          int planning_lane = changing_lanes ? target_lane : lane;
        
          // Add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30, 2+4*planning_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, 2+4*planning_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, 2+4*planning_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
        
          for (int i = 0; i < ptsx.size(); i++) {
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
            ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
          }
        
          // Create a spline
          tk::spline s;
          // Set (x,y) points to the spline (i.e. fits a spline to those points)
          s.set_points(ptsx, ptsy);
          
          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // Calculate how to break up spline points so that we travel at desired velocity
          double target_x = 30.0; // 30.0 m is the distance horizon
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y); // this is the d in the diagram
          double x_add_on = 0.0; // Related to the transformation (starting at zero)
          // Fill up the rest of path planner after filling it with previous points, will always output 50 points
          for (int i = 1; i <= 50-previous_path_x.size(); i++) {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate x, y back to normal
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

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
