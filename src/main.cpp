#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Spline.h"
#include "json.hpp"

using namespace std;

const int LANE_WIDTH = 4;
const int NUM_LANES = 3;
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
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

double get_lane_d(int l) { return (LANE_WIDTH / 2.0) + LANE_WIDTH * l; }
int get_lane_num(double d) {
  double a = ((d - ((double)LANE_WIDTH / 2)) / (double)LANE_WIDTH);
  return round(a);
}

vector<int> get_lane_options(int current_lane) {
  vector<int> lane_options;
  if (current_lane > 0 && current_lane < (NUM_LANES - 1)) {
    lane_options.push_back(current_lane - 1);
    lane_options.push_back(current_lane + 1);
  } else if (current_lane == 0) {
    lane_options.push_back(current_lane + 1);
  } else if (current_lane == (NUM_LANES - 1)) {
    lane_options.push_back(current_lane - 1);
  }
  return lane_options;
}

// creating my own namespace to not interfere with json or others
namespace pp {

// Helpers
struct Point {
  Point(){};
  Point(double ptX, double ptY) {
    X = ptX;
    Y = ptY;
  };
  double X;
  double Y;
};

// Finite state machines and cost functions
enum fsm {
  KEEP_LANE,
  PREP_LANE_CHANGE,
  CHANGE_LANE,
};

double to_mph(double speed_in_meters_per_sec) {
  return speed_in_meters_per_sec * 2.24;
}

double to_mps(double speed_in_mph) { return speed_in_mph / 2.24; }

// cost of not going speed limit
double get_speed_cost(double speed_limit, double new_speed) {
  double cost;
  if (new_speed >= speed_limit || new_speed == -1) {
    cost = 0;
  } else {
    cost = (speed_limit - new_speed) / speed_limit;
  }
  return cost;
}

// true = safe
bool safe_to_change_lanes(int intended_lane, double safe_distance,
                          double current_s, std::vector<int> close_cars,
                          vector<vector<double>> sensor_data) {
  vector<int>::iterator it;
  for (it = close_cars.begin(); it != close_cars.end(); ++it) {
    // check if the car is close enough an in the intended lane
    if ((abs(current_s - sensor_data[*it][5]) < safe_distance) &&
        (get_lane_num(sensor_data[*it][6]) == intended_lane)) {
      printf("car within proximity: unsafe to change to lane %i\n",
             intended_lane);
      return false;
    }
  }
  return true;
}

double get_inefficiency_cost() {
  // placeholder
  return 0;
}
}  // namespace pp

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closestLen = 100000;  // large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
pp::Point getXY(double s, double d, const vector<double> &maps_s,
                const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading =
      atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return pp::Point(x, y);
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

  double speed_limit = 49.5;
  double ref_speed = .224;
  double lane = get_lane_d(1);
  double radius = 30;
  double follow_dist = radius * 1.25;
  bool lane_change_in_progress = false;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &ref_speed, &speed_limit,
               &lane, &radius, &follow_dist, &lane_change_in_progress](
                  uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          pp::Point car_xy(j[1]["x"], j[1]["y"]);
          double current_car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          size_t prev_size = previous_path_x.size();

          // use the end of the last path as the car's s
          double end_path_car_s;
          if (prev_size > 0) {
            end_path_car_s = end_path_s;
          } else {
            end_path_car_s = current_car_s;
          }

          bool too_close = false;
          bool too_slow = false;
          bool within_follow_dist = false;
          float closest_to_ego = 999;

          vector<int> close_cars;
          double lane_speed[3] = {-1, -1, -1};
          int car_in_path = -1;
          double car_in_path_speed = -1;

          // if there is a lane change in progress
          if (lane_change_in_progress &&
              get_lane_num(car_d) == get_lane_num(lane)) {
            lane_change_in_progress = false;
            printf("\n**lane change complete**\n\n");
          }

          // find all the cars that are around ego
          for (size_t i = 0; i < sensor_fusion.size(); ++i) {
            float their_d = sensor_fusion[i][6];
            float their_s = sensor_fusion[i][5];
            // get the other car's lane number
            int ln = get_lane_num(their_d);
            // get the other car's velocity (in the x and y directions)
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            // calculate magnitude of the x,y directional speeds
            double their_vel = sqrt(pow(vx, 2) + pow(vy, 2));
            // project out where the other car is
            double check_car_s = their_s;
            check_car_s += ((double)prev_size * .02 * their_vel);
            bool in_front_of_ego = (their_s >= current_car_s);
            // lane speed is the slowest car in that lane that's ahead of ego
            if (ln != -1) {
              if (lane_speed[ln] == -1 && in_front_of_ego) {
                lane_speed[ln] = pp::to_mph(their_vel);
              }
            }
            // check if car is in ego's lane
            if (ln == get_lane_num(lane)) {
              // check that it is infront of ego
              if (in_front_of_ego) {
                if ((their_s - current_car_s) < closest_to_ego) {
                  car_in_path_speed = pp::to_mph(their_vel);
                  closest_to_ego = their_s - current_car_s;
                }
                car_in_path = sensor_fusion[i][0];
                // they are too close
                if ((check_car_s - end_path_car_s) < radius) {
                  too_close = true;
                } else if ((check_car_s - end_path_car_s) < follow_dist) {
                  within_follow_dist = true;
                }
                // check if they are going slower than ego
                if (car_in_path_speed < speed_limit &&
                    ((check_car_s - end_path_car_s) < follow_dist)) {
                  too_slow = true;
                }
              }
            }
            // check for other cars close to ego
            if (abs(their_s - end_path_car_s) < radius ||
                abs(their_s - current_car_s) < radius) {
              close_cars.push_back(sensor_fusion[i][0]);
            }
          }

          vector<int> lane_options = get_lane_options(get_lane_num(lane));

          // if the car infront of ego is going to slow determine costs of
          // changing lanes before it has to slow down
          if (too_slow) {
            double speed_cost =
                pp::get_speed_cost(speed_limit, car_in_path_speed);
            double speed_cost_for_intended_lane = 1;
            for (vector<int>::iterator it = lane_options.begin();
                 it != lane_options.end(); ++it) {
              int intended_lane_num = *it;
              speed_cost_for_intended_lane = pp::get_speed_cost(
                  speed_limit, lane_speed[intended_lane_num]);
              printf("speed_cost_for_intended_lane %i = %f\n",
                     intended_lane_num, speed_cost_for_intended_lane);
              printf("speed_cost = %f\n", speed_cost);
              if (speed_cost_for_intended_lane < speed_cost) {
                if (lane_change_in_progress) {
                  printf("lane change in progress\n");
                } else if (pp::safe_to_change_lanes(
                               intended_lane_num, radius / 2, current_car_s,
                               close_cars, sensor_fusion)) {
                  lane_change_in_progress = true;
                  lane = get_lane_d(intended_lane_num);
                  printf("Safe to change to lane %i\n", intended_lane_num);
                  break;
                }
              }
            }
          }

          // if ego is too close slow down faster than if they are within the
          // follow distance
          if (too_close) {
            ref_speed -= .224;
          } else if (within_follow_dist) {
            if (ref_speed > car_in_path_speed) {
              ref_speed -= .224 / 2;
            } else if (ref_speed < car_in_path_speed - .488) {
              ref_speed += .224 * 3 / 4;
            } else {
              ref_speed = car_in_path_speed;
            }
          } else if (ref_speed < speed_limit - 1) {
            ref_speed += .224 * 2;
          } else {
            ref_speed = speed_limit;
          }

          json msgJson;

          vector<pp::Point> ptsxy;

          pp::Point ref_xy = car_xy;
          double ref_yaw = deg2rad(car_yaw);

          // if we are just starting out, use the car's position, otherwise get
          // the last two points and get the orientation of the car in line to
          // those two points.
          if (prev_size < 2) {
            pp::Point prev_car_xy(car_xy.X - cos(car_yaw),
                                  car_xy.Y - sin(car_yaw));
            ptsxy.push_back(prev_car_xy);
            ptsxy.push_back(car_xy);
          } else {
            ref_xy.X = previous_path_x[prev_size - 1];
            ref_xy.Y = previous_path_y[prev_size - 1];
            pp::Point ref_xy_prev(previous_path_x[prev_size - 2],
                                  previous_path_y[prev_size - 2]);
            ref_yaw = atan2(ref_xy.Y - ref_xy_prev.Y, ref_xy.X - ref_xy_prev.X);
            ptsxy.push_back(ref_xy_prev);
            ptsxy.push_back(ref_xy);
          }

          // how far out we are going to lay our path our for
          pp::Point next_wp0 =
              getXY(end_path_car_s + radius, lane, map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          pp::Point next_wp1 =
              getXY(end_path_car_s + radius * 2, lane, map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          pp::Point next_wp2 =
              getXY(end_path_car_s + radius * 3, lane, map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          ptsxy.push_back(next_wp0);
          ptsxy.push_back(next_wp1);
          ptsxy.push_back(next_wp2);

          for (size_t i = 0; i < ptsxy.size(); ++i) {
            pp::Point shift(ptsxy[i].X - ref_xy.X, ptsxy[i].Y - ref_xy.Y);
            ptsxy[i] = pp::Point(
                (shift.X * cos(0 - ref_yaw)) - (shift.Y * sin(0 - ref_yaw)),
                (shift.X * sin(0 - ref_yaw)) + (shift.Y * cos(0 - ref_yaw)));
          }

          // setup the spline
          tk::spline s;
          vector<double> x_pts;
          vector<double> y_pts;
          auto to_x_and_y_vectors = [&x_pts, &y_pts](const pp::Point &i) {
            x_pts.push_back(i.X);
            y_pts.push_back(i.Y);
          };
          for_each(ptsxy.begin(), ptsxy.end(), to_x_and_y_vectors);
          s.set_points(x_pts, y_pts);

          // clear out the vectors for use later
          x_pts.clear();
          y_pts.clear();

          // add the previous path to our current
          for (size_t i = 0; i < prev_size; ++i) {
            x_pts.push_back(previous_path_x[i]);
            y_pts.push_back(previous_path_y[i]);
          }

          // target_x is the x for how far out each segment is projecting
          double target_x = radius;
          // target_y is the y of the point at that target_x projection
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
          double x_add_on = 0;
          double N = target_dist / (.02 * pp::to_mps(ref_speed));

          for (size_t i = 0; i <= 50 - prev_size; ++i) {
            // add a point on the spline
            pp::Point pt_on_s(x_add_on + (target_x / N),
                              s(x_add_on + (target_x / N)));
            x_add_on = pt_on_s.X;

            pp::Point ref = pt_on_s;
            // correct it to be in the map's reference frame
            pt_on_s.X = ref.X * cos(ref_yaw) - ref.Y * sin(ref_yaw);
            pt_on_s.Y = ref.X * sin(ref_yaw) + ref.Y * cos(ref_yaw);

            pt_on_s.X += ref_xy.X;
            pt_on_s.Y += ref_xy.Y;
            x_pts.push_back(pt_on_s.X);
            y_pts.push_back(pt_on_s.Y);
          }

          // Define a path made up of (x,y) points that the car will
          // visit sequentially every .02 seconds
          msgJson["next_x"] = x_pts;
          msgJson["next_y"] = y_pts;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
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
