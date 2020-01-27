#pragma once 

#include <map>
#include <vector>

#include "json.hpp"
#include "car.h"

using std::vector;

using prev_path_t = nlohmann::basic_json<std::map, std::vector, std::basic_string<char, std::char_traits<char>, std::allocator<char> >, bool, long long, unsigned long long, double, std::allocator, nlohmann::adl_serializer>;

class Planner {

public:
	Planner(
		vector<double> map_waypoints_x,
		vector<double> map_waypoints_y,
		vector<double> map_waypoints_s,
		vector<double> map_waypoints_dx,
		vector<double> map_waypoints_dy,
		double max_velocity,
		int init_lane,
		double init_velocity
	);

std::pair<vector<double>, vector<double>> plan(
		double car_x,
		double car_y,
		double car_s,
		double car_d,
		double car_yaw,
		double car_speed,
		prev_path_t previous_path_x,
		prev_path_t previous_path_y,
		double end_path_s,
		double end_path_d,
		vector<vector<double>> sensor_fusion
	);

private:

vector<Car> closest_cars_in_lane(
	int lane,
	double car_s,
	int prev_size,
	vector<vector<double>> sensor_fusion
);

int fastest_safe_lane(
	int current_lane,
	double car_s,
	int prev_size,
	vector<vector<double>> sensor_fusion
);

    const vector<double> map_waypoints_x;
	const vector<double> map_waypoints_y;
	const vector<double> map_waypoints_s;
	const vector<double> map_waypoints_dx;
	const vector<double> map_waypoints_dy;
	const double max_velocity;
	int lane;
	double velocity;
	bool lane_change_in_progress;
	int steps_since_lane_change_start;
};