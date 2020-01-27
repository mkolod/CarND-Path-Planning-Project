#include "math.h"
#include <iostream>
#include <optional>
#include <vector>

#include "car.h"
#include "helpers.h"
#include "planner.h"
#include "spline.h"

using std::vector;

Planner::Planner(vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                 vector<double> map_waypoints_s,
                 vector<double> map_waypoints_dx,
                 vector<double> map_waypoints_dy, double max_velocity,
                 int init_lane, double init_velocity)
    : map_waypoints_x(map_waypoints_x), map_waypoints_y(map_waypoints_y),
      map_waypoints_s(map_waypoints_s), map_waypoints_dx(map_waypoints_dx),
      map_waypoints_dy(map_waypoints_dy), max_velocity(max_velocity),
      lane(init_lane), velocity(init_velocity) {}

vector<Car>
Planner::closest_cars_in_lane(int req_lane, double car_s, int prev_size,
                              vector<vector<double>> sensor_fusion) {

  vector<Car> in_front;
  vector<Car> behind;

  for (int i = 0; i < sensor_fusion.size(); ++i) {
    float d = sensor_fusion[i][6];
    // If car is within the requested lane
    if (d < (2 + 4 * req_lane + 2) && d > (2 + 4 * req_lane - 2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += static_cast<double>(prev_size * .02 * check_speed);

      Car check_car(vx, vy, check_car_s);
      if (check_car_s > car_s) {
        in_front.push_back(check_car);
      } else {
        behind.push_back(check_car);
      }
    }
  }

  // Find the car with the smallest s in front
  std::sort(in_front.begin(), in_front.end(),
            [](const Car &lhs, const Car &rhs) { return lhs.s < rhs.s; });
  // Find the car with the largest s in the back
  std::sort(behind.begin(), behind.end(),
            [](const Car &lhs, const Car &rhs) { return lhs.s > rhs.s; });

  // std::cout << "\n### lane = " << req_lane << std::endl;
  // std::cout << "Cars in front: " << std::endl;
  // for (const auto& car : in_front) {
  //   std::cout << car.s << " ";
  // }
  // std::cout << std::endl;
  // std::cout << "Cars in back: " << std::endl;
  // for (const auto& car : behind) {
  //   std::cout << car.s << " ";
  // }
  // std::cout << "My car s = " << car_s << std::endl << std::endl;

  vector<Car> result;

  if (behind.size() > 0) {
    result.push_back(behind[0]);
  }

  if (in_front.size() > 0) {
    result.push_back(in_front[0]);
  }

  return result;
}


int Planner::fastest_safe_lane(
  int current_lane,
  double car_s,
  int prev_size,
  vector<vector<double>> sensor_fusion
) {

  int best_lane = current_lane;
  double best_speed = 0.0;

  vector<int> lanes;
  switch (current_lane) {
    case 0:
      lanes = {0, 1};
      break;
    case 1:
      lanes = {0, 1, 2};
      break;
    case 2:
      lanes = {1, 2};
      break;
  } 

  for (const auto& lane : lanes) {
    vector<Car> cars_in_lane = closest_cars_in_lane(lane, car_s, prev_size, sensor_fusion);
    bool safe = true;
    double speed = 0.0;
    int counter = 0;
    for (const auto& car: cars_in_lane) {
      double safe_distance = physical_braking_distance(car.speed);
      if (abs(car_s - car.s) < safe_distance) {
        safe = false;
      }
      if (car.s > car_s && (car.s - car_s) < 100) {
        speed += mps_to_mph(car.speed);
        counter += 1;
      }
    }
    if (counter > 0) {
        speed /= counter;
    }
    if (safe && (speed > best_speed)) {
      best_speed = speed;
      best_lane = lane;
    }
  }
  return best_lane;
}

std::pair<vector<double>, vector<double>>
Planner::plan(double car_x, double car_y, double car_s, double car_d,
              double car_yaw, double car_speed, prev_path_t previous_path_x,
              prev_path_t previous_path_y, double end_path_s, double end_path_d,
              vector<vector<double>> sensor_fusion) {
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  int prev_size = previous_path_x.size();

  // Avoid hitting car in front
  // Do this in Frenet coordinates, since it's easier
  if (prev_size > 0) {
    car_s = end_path_s;
  }

  bool too_close = false;

  // Find ref_v to use
  for (int i = 0; i < sensor_fusion.size(); ++i) {
    // Car is in my lane
    float d = sensor_fusion[i][6];
    if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];
      // If using previous points, can project s value out.
      check_car_s += static_cast<double>(prev_size * .02 * check_speed);
      // Check s values greater than mine and s gap

      double safe_distance = physical_braking_distance(mph_to_mps(velocity));

      if (check_car_s > car_s && ((check_car_s - car_s) < safe_distance)) {
        // Do some logic here, lower reference velocity so we don't
        // crash
        // into the car in front of us, could also try to change lanes.
        too_close = true;
        std::cout << "Distance = " << abs(check_car_s - car_s)
                  << ", safe distance = " << safe_distance << std::endl;
      }
    }
  }

  // for (int l = 0; l < 3; ++l) {
  //   std::cout << "Lane " << l << ". Closest cars: " << std::endl;
  //   vector<Car> v = closest_cars_in_lane(l, car_s, prev_size, sensor_fusion);
  //   for (const auto &elem : v) {
  //     std::cout << "speed = " << mps_to_mph(elem.speed) << ", s = " << elem.s << ", ";
  //     std::cout << "distance = " << abs(car_s - elem.s) << ", ";
  //     std::cout << (elem.s < car_s ? "behind" : "in front");
  //     std::cout << ", my_car_s = " << car_s << std::endl;
  //   }
  // }

  // std::cout << "My velocity: " << velocity << std::endl;

  if (too_close) {
    velocity -= .224;
  } else if (velocity < 49.5) {
    velocity += .224;
  }

  // Use 49.0 to allow for margin of error
  if (velocity < 49.0) {
    bool can_change_lane = true;
    int best_lane = fastest_safe_lane(lane, car_s, prev_size, sensor_fusion);
    if (best_lane != lane) {
      if (!lane_change_in_progress) {
        lane_change_in_progress = true;
        lane = best_lane;
        steps_since_lane_change_start = 0;
      } else if (lane_change_in_progress) {
        if (steps_since_lane_change_start > 500) {
          lane_change_in_progress = false;
        }
      }
    }
  }

  steps_since_lane_change_start += prev_size;

/*
int fastest_safe_lane(
  int current_lane,
  double car_s,
  int prev_size,
  vector<vector<double>> sensor_fusion
*/

  // Create a list of widely spaced waypoints, evenly spaced at 30m.
  // Later we will interpolate these waypoints with a spline
  // and fill it in with more points.
  vector<double> ptsx;
  vector<double> ptsy;

  // Reference x, y, yaw states.
  // Either we will reference the starting point as where the car
  // is at, or at the previous paths and point.
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // If previous size is almost empty, use the car as starting
  // reference.
  if (prev_size < 2) {
    // Use two points that make the path tangent to the car.
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);

  } else { // Use the previous path's end point as a starting reference
    // Redefine reference state as previous path and point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly 30m points ahead of the starting reference
  vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s,
                                  map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s,
                                  map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s,
                                  map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); ++i) {
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  // Create a spline
  tk::spline s;

  // Set (x, y) points to the spline
  s.set_points(ptsx, ptsy);

  // Start with all of the previous points from last time
  for (int i = 0; i < previous_path_x.size(); ++i) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our
  // desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0.0;

  // Fill up the rest of our path planner after filling it with previous
  // points.
  for (int i = 1; i <= 50 - previous_path_x.size(); ++i) {
    double N = target_dist / (.02 * velocity / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);
    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  return std::make_pair(next_x_vals, next_y_vals);
}

