#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// Define shortcut
using json = nlohmann::json;

// For converting back and forth between radians and degrees
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format it will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; // Large number
	int closestWaypoint = 0;

  // Find the closest waypoint
	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}
	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

  // Find the closest waypoint
	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  // Find
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];
	double heading = atan2((map_y-y), (map_x-x));
	double angle = abs(theta - heading);
	if(angle > pi()/4)
	{
		closestWaypoint++;
	}
	return closestWaypoint;
}

// Transform from Cartesian x, y coordinates to Frenet s, d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// Find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;
	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	// See if d value is positive or negative by comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// Calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
	}
	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};

}

// Transform from Frenet s, d coordinates to Cartesian x, y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));

	// Get the x, y, s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x, y};
}

bool check_lane(int check_lane, double car_s, vector<vector<double>> sensor_fusion) {
  // Check to see if a car is in the lane
  bool car = false;
  for(int j = 0; j < sensor_fusion.size(); j++) {
    double check_car_s = sensor_fusion[j][5]; // 5 is frenet 's' value
    double check_car_d = sensor_fusion[j][6]; // 6 is frenet 'd' value
    double check_d = 2 + 4 * check_lane;

    // Is a car within that lane?
    if(check_car_s > car_s - 10.0 && check_car_s < car_s + 20.0) {
      if(check_car_d > check_d - 2 && check_car_d < check_d + 2) {
        printf("Oh noes.\n");
        car = true;
      }
    }
  }
  return car;
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  // Which lane are we in?
  int current_lane = 1;

  // How fast should we be going as factor of speed limit?
  double target_speed_factor = 1.0;
  double current_speed_factor = 0.1;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &current_lane, &target_speed_factor, &current_speed_factor](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

//    auto sdata = string(data).substr(0, length);
//    cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
            // j[1] is the data JSON object
          
          	// Main car's localization data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
            cout << "X: " << car_x << "  Y: " << car_y << endl;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            // How much of the previous path is there still to go?
            int previous_path_size = previous_path_x.size();

            // Our waypoints
            vector<double> waypoints_x;
            vector<double> waypoints_y;

            // Our reference
            double reference_x = car_x;
            double reference_y = car_y;
            double reference_yaw = deg2rad(car_yaw);

            // If just starting out
            if( previous_path_size < 2 ) {
              double previous_car_x = car_x - cos(car_yaw);
              double previous_car_y = car_y - sin(car_yaw);

              // Our waypoints has two points, one where the car is, one where it was.
              waypoints_x.push_back(previous_car_x);
              waypoints_y.push_back(previous_car_y);
              waypoints_x.push_back(car_x);
              waypoints_y.push_back(car_y);
            }
            else {
              reference_x = previous_path_x[previous_path_size-1];
              reference_y = previous_path_y[previous_path_size-1];
              double reference_x_previous = previous_path_x[previous_path_size-2];
              double reference_y_previous = previous_path_y[previous_path_size-2];

              // Calculate yaw
              reference_yaw = atan2(reference_y - reference_y_previous, reference_x - reference_x_previous);

              // Our waypoints has two points, one where the car is, one where it was.
              waypoints_x.push_back(reference_x_previous);
              waypoints_y.push_back(reference_y_previous);
              waypoints_x.push_back(reference_x);
              waypoints_y.push_back(reference_y);
            }

            // Add points 30m, 60m, and 90m out.
            vector<double> next_waypoint_0 = getXY(car_s+30, (2+4*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_waypoint_1 = getXY(car_s+60, (2+4*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_waypoint_2 = getXY(car_s+90, (2+4*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            waypoints_x.push_back(next_waypoint_0[0]);
            waypoints_y.push_back(next_waypoint_0[1]);
            waypoints_x.push_back(next_waypoint_1[0]);
            waypoints_y.push_back(next_waypoint_1[1]);
            waypoints_x.push_back(next_waypoint_2[0]);
            waypoints_y.push_back(next_waypoint_2[1]);

            // Recenter to car coords
            for(int i = 0; i < waypoints_x.size(); i++) {
              double shift_x = waypoints_x[i] - reference_x;
              double shift_y = waypoints_y[i] - reference_y;
              waypoints_x[i] = (shift_x * cos(-reference_yaw) - shift_y * sin(-reference_yaw));
              waypoints_y[i] = (shift_x * sin(-reference_yaw) + shift_y * cos(-reference_yaw));
            }

            // If our speed is too slow, speed up
            if(current_speed_factor < target_speed_factor) {
              current_speed_factor += 0.01;
            }
            else if(current_speed_factor > target_speed_factor) {
              current_speed_factor -= 0.01;
            }

            // Our spline
            tk::spline s;
            s.set_points(waypoints_x, waypoints_y);

            // Plot the path
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // Load up the future path with what's left of the previous path
            for(int i = 0; i < previous_path_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);              
            }

            // Render spline values at positions to get target speed and spliney trajectory
            double horizon = 30.0;
            double horizon_y = s(horizon); // Calculate spline value by just calling s().
            double horizon_distance = sqrt(horizon*horizon + horizon_y*horizon_y); // Good old Pythag

            // Split our distance to horizon into n points 
            double n = horizon_distance / (0.02 * current_speed_factor * 49.0/2.24);
            double x_so_far = 0;
            for(int i = 0; i <= 50 - previous_path_size; i++) {

              // Caluclate this point
              double x_point = x_so_far + horizon/n;
              double y_point = s(x_point);
              x_so_far = x_point;

              // Rotate back to world coords
              double x_ref = x_point;
              double y_ref = y_point;
              x_point = (x_ref * cos(reference_yaw) - y_ref * sin(reference_yaw));
              y_point = (x_ref * sin(reference_yaw) + y_ref * cos(reference_yaw));
              x_point += reference_x;
              y_point += reference_y;

              // Add 'em
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            // Check sensor fusion for other cars
            target_speed_factor = 1.0;
            for(int i = 0; i < sensor_fusion.size(); i++) {
              double other_car_s = sensor_fusion[i][5]; // 5 is frenet 's' value
              double other_car_d = sensor_fusion[i][6]; // 6 is frenet 'd' value
              double our_car_d = 2 + 4 * current_lane;

              // Is the other car within our lane?
              if(other_car_d > our_car_d - 2 && other_car_d < our_car_d + 2) {

                // Get its speed
//                double other_car_vx = sensor_fusion[i][3];
//                double other_car_vy = sensor_fusion[i][4];
//                double other_car_speed = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);

                // Where will the other car be next frame?
                //other_car_s += ((double)previous_path_size * 0.02 * other_car_speed);

                // Check the car's s position, is it way too close up front?

                // Check the car's s position, is it too close up front?
                if(other_car_s > car_s && other_car_s < car_s + 40.0) {

                  // Slow down
                  target_speed_factor = 0.5;

                  // Really slow down
                  if(other_car_s < car_s + 20.0) {

                    // Slow down
                    target_speed_factor = 0.2;
                  }

                  // If we're in the middle lane, check either side
                  if(current_lane == 1) {

                    // Check left and right lanes, make sure nothing is in the lane
                    bool car = check_lane(0, car_s, sensor_fusion);
                    if(!car){
                      current_lane = 0;
                    }
                    else {
                      bool car = check_lane(2, car_s, sensor_fusion);
                      if(!car){
                        current_lane = 2;
                      }
                      else{
                        printf("Stuck behind a wall of slowpokes.\n");
                      }
                    }
                  }
                  else if(current_lane == 0){
                    // Check middle lane
                    bool car = check_lane(1, car_s, sensor_fusion);
                    if(!car){
                      current_lane = 1;
                    }
                  }
                  else if(current_lane == 2){
                    // Check middle lane
                    bool car = check_lane(1, car_s, sensor_fusion);
                    if(!car){
                      current_lane = 1;
                    }
                  }
                }
              }
            }

            // Send 
            json msgJson;
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;
          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });


  // -------------------------------------------------------------------------
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      res->end(nullptr, 0);
    }
  });

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
