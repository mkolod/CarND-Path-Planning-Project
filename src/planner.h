#pragma once 

#include <vector>

using std::vector;

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

private:
    const vector<double> map_waypoints_x;
	const vector<double> map_waypoints_y;
	const vector<double> map_waypoints_s;
	const vector<double> map_waypoints_dx;
	const vector<double> map_waypoints_dy;
	const double max_velocity;
	int lane;
	double velocity;
};