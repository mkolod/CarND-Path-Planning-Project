#pragma once

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Convert miles per hour to meters per second
double mph_to_mps(double mph);

// Braking distance based on physics
// See here: https://en.wikipedia.org/wiki/Braking_distance
// Velocity is in m/s
// Default friction coefficient is on dry asphalt
// Default gravity is Earth's gravity (m/s^2)
// Returns: braking distance in meters
double physical_braking_distance(double velocity, double friction_coeff=0.7, double gravity=9.81);


// Distance traveled due to non-instantaneous reaction
// Velocity is in m/s
// Reaction time is in s
double reaction_distance(double velocity, double reaction_time);


// Sum of reaction distance and physical braking distance
// Velocity is in m/s
// Reaction time is in s
// Gravity is in m/s^2
// Returns: distance in m
double total_braking_distance(double velocity, double reaction_time, double friction_coeff=0.7, double gravity=9.81);


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y);