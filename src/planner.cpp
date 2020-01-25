#include "helpers.h"
#include "planner.h"

Planner::Planner(
	vector<double> map_waypoints_x,
	vector<double> map_waypoints_y,
	vector<double> map_waypoints_s,
	vector<double> map_waypoints_dx,
	vector<double> map_waypoints_dy,
	double max_velocity,
	int init_lane,
	double init_velocity
) :
map_waypoints_x(map_waypoints_x),
map_waypoints_y(map_waypoints_y),
map_waypoints_s(map_waypoints_s),
map_waypoints_dx(map_waypoints_dx),
map_waypoints_dy(map_waypoints_dy),
max_velocity(max_velocity),
lane(init_lane),
velocity(init_velocity)
{
}

// class Planner {

// private:
//     const vector<double> map_waypoints_x;
// 	const vector<double> map_waypoints_y;
// 	const vector<double> map_waypoints_s;
// 	const vector<double> map_waypoints_dx;
// 	const vector<double> map_waypoints_dy;
// 	const double max_velocity;
// 	int lane;
// 	double velocity;
// };

