#pragma once
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

typedef vector<Vehicle::Snapshot>::iterator vec_iter;

struct TrajectoryData {

	int proposed_lane;
	float avg_speed;
	float max_accel;
	float rms_acceleration;
	float closest_approach;
	float end_distance_to_goal;
	float end_lanes_from_goal;
	map<string, int> collides;
};

double change_lane_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, struct TrajectoryData data);

double distance_from_goal_lane(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, TrajectoryData data);

double inefficiency_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, TrajectoryData data);

double collision_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, TrajectoryData data);

double buffer_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, TrajectoryData data);

double calculate_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, bool verbose);

map<int, Vehicle::Snapshot > filtTraj(vec_iter first, vec_iter last);

TrajectoryData get_helper_data(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions);

bool check_collision(Vehicle::Snapshot snapshot, double s_previous, double s_now);

vector<double> unpack_snapshot(Vehicle::Snapshot snapshot);

map<int, vector<vector<double> > > filter_predictions_by_lane(map<int, vector<vector<double> > > predictions, int lane);