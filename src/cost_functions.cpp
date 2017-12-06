#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include <limits>
#include <cassert>
#include "vehicle.h"
#include "cost_functions.h"

TrajectoryData datas;

//priority levels for costs
double COLLISION = pow(10, 6);
double DANGER = pow(10, 5);
double REACH_GOAL = pow(10, 5);
double COMFORT = pow(10, 4);
double EFFICIENCY = pow(10, 2);

float DESIRED_BUFFER = 1.5;  // timesteps
int PLANNING_HORIZON = 2;

//bool DEBUG = false;
bool DEBUG = true;

double change_lane_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, TrajectoryData data) {
	float proposed_lanes = data.end_lanes_from_goal;
	int cur_lanes = trajectory[0].lane;
	double cost = 0;
	if (proposed_lanes > cur_lanes) {
		cost = COMFORT;
	}
	if (proposed_lanes < cur_lanes) {
		cost = -COMFORT;
	}
	if (cost != 0) {
		//cout << "!! \n\ncost for lane change is " << cost << "\n\n" << endl;
	}
	return cost;
}

double distance_from_goal_lane(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, TrajectoryData data) {
	float distance = abs(data.end_distance_to_goal);
	distance = max(distance, float(1.0));
	float time_to_goal = distance / data.avg_speed;
	float lanes = data.end_lanes_from_goal;
	float multiplier = float(5 * lanes / time_to_goal);
	double cost = multiplier * REACH_GOAL;
	return cost;
}

double inefficiency_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, TrajectoryData data) {
	float speed = data.avg_speed;
	double target_speed = vehicle.target_speed;
	float diff = target_speed - speed;
	float pct = diff / target_speed;
	float multiplier = pow(pct, 2);
	double cost = multiplier * EFFICIENCY;
	return cost;
}

double collision_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, TrajectoryData data) {
	if (!data.collides.empty()) {
		int time_til_collision = data.collides["at"];
		float exponent = pow(float(time_til_collision), 2);
		float multiplier = exp(-exponent);

		double cost = 1 * COLLISION;
		return cost;
	}
	return 0;
}

double buffer_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, TrajectoryData data) {
	float closest = data.closest_approach;
	if (closest == 0) {
		return 10 * DANGER;
	}

	float timesteps_away = closest / data.avg_speed;

	if (timesteps_away > DESIRED_BUFFER) {
		return 0.0;
	}

	float multiplier = float(1.0) - pow((timesteps_away / DESIRED_BUFFER), 2);
	return multiplier * DANGER;
}

double calculate_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions, bool verbose = true) {
	TrajectoryData trajectory_data = get_helper_data(vehicle, trajectory, predictions);
	double cost = 0.0;
	double change_cost = change_lane_cost(vehicle, trajectory, predictions, trajectory_data);
	// cout<<"change_cost "<<change_cost<<endl;
	double distance_c = distance_from_goal_lane(vehicle, trajectory, predictions, trajectory_data);
	// cout<<"distance_c "<<distance_c<<endl;
	double inefficiency_c = inefficiency_cost(vehicle, trajectory, predictions, trajectory_data);
	// cout<<"inefficiency_c "<<inefficiency_c<<endl;
	double collision_c = collision_cost(vehicle, trajectory, predictions, trajectory_data);
	// cout<<"collision_c "<<collision_c<<endl;
	double buffer_c = buffer_cost(vehicle, trajectory, predictions, trajectory_data);
	// cout<<"buffer_c "<<buffer_c<<endl;
	cost = change_cost + distance_c + inefficiency_c + collision_c + buffer_c;
	// cout<<"cost "<<cost<<endl;
	return cost;
}

map<int, Vehicle::Snapshot > filtTraj(vec_iter first, vec_iter last) {
	map<int, Vehicle::Snapshot > temp_vec;
	int i = 1;
	for (vec_iter cur = first; cur != last; ++cur) {
		//std::cout << "cur[0].s " << cur[0].s << endl;
		temp_vec[i] = cur[0];
		i++;
	}
	return temp_vec;
}

TrajectoryData get_helper_data(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int, vector<vector<double> > > predictions) {
	vector<Vehicle::Snapshot> t = trajectory;
	Vehicle::Snapshot current_snapshot = t[0];
	Vehicle::Snapshot first = t[1];
	Vehicle::Snapshot last = t.back();
	double end_distance_to_goal = vehicle.goal_s - last.s;
	double end_lanes_from_goal = abs(vehicle.goal_lane - last.lane);
	float dt = trajectory.size();
	double proposed_lane = first.lane;
	float avg_speed = (last.s - current_snapshot.s) / dt;

	vector<float> accels = {};
	double closest_approach = numeric_limits<double>::infinity();
	map<string, int> collides;
	Vehicle::Snapshot last_snap = trajectory[0];
	map<int, vector<vector<double> > > filtered = filter_predictions_by_lane(predictions, proposed_lane);

	map<int, Vehicle::Snapshot > filtered_trajectory = filtTraj(trajectory.begin() + 1, trajectory.begin() + PLANNING_HORIZON + 1);

	map<int, Vehicle::Snapshot >::iterator it = filtered_trajectory.begin();
	vector<vector<vector<double> > > at_behind;
	while (it != filtered_trajectory.end())
	{
		int v_id = it->first;
		Vehicle::Snapshot v = it->second;

		vector<double> unpck_snap = unpack_snapshot(filtered_trajectory[v_id]);

		accels.push_back(unpck_snap[4]);

		map<int, vector<vector<double> > >::iterator it2 = filtered.begin();
		vector<vector<vector<double> > > at_behind;
		while (it2 != filtered.end())
		{
			int v2_id = it2->first;
			vector<vector<double> > v2 = it2->second;

			vector<double> state = v2[v_id];
			vector<double> last_state = v2[v_id - 1];
			bool vehicle_collides = check_collision(filtered_trajectory[v_id], last_state[1], state[1]);

			if (vehicle_collides) {
				collides["at"] = v_id;
			}
			double dist = abs(state[1] - unpck_snap[1]);
			if (dist < closest_approach) {
				closest_approach = dist;
			}

			it2++;
		}
		last_snap = filtered_trajectory[v_id];

		it++;
	}
	float max_accel = *max_element(accels.begin(), accels.end());
	vector<float> rms_accels = {};
	for (int i = 0; i < accels.size(); i++) {
		rms_accels.push_back(pow(accels[i], 2));
	}
	int num_accels = accels.size();
	float rms_acceleration = 0;
	for (auto& n : rms_accels) rms_acceleration += n;
	rms_acceleration /= num_accels;

	datas.proposed_lane = proposed_lane;
	datas.avg_speed = avg_speed;
	datas.max_accel = max_accel;
	datas.rms_acceleration = rms_acceleration;
	datas.closest_approach = closest_approach;
	datas.end_distance_to_goal = end_distance_to_goal;
	datas.end_lanes_from_goal = end_lanes_from_goal;
	datas.collides = collides;

	return datas;
}

bool check_collision(Vehicle::Snapshot snapshot, double s_previous, double s_now) {
	double s = snapshot.s;
	double v = snapshot.v;
	double v_target = s_now - s_previous;
	// cout<<"s_previous "<<s_previous<<endl;
	// cout<<"s_now "<<s_now<<endl;
	// cout<<"s "<<s<<endl;
	// cout<<"lane "<<snapshot.lane<<endl;
	// cout<<"state "<<snapshot.state<<endl;
	int test = 15;
	if(s_now < s-5 && s_now > s-25) {
		// if(snapshot.state != "KL") {
		// 	cout<<"true "<<snapshot.state<<endl;
		// }
		return true;
	}
	else {
		return false;
	}
	// if(snapshot.state == "KL") {
	// 	if (s_previous < s) {
	// 		if (s_now >= s) {
	// 			return true;
	// 		}
	// 		else {
	// 			return false;
	// 		}
	// 	}
	// 	if (s_previous > s) {
	// 		if (s_now <= s) {
	// 			return true;
	// 		}
	// 		else {
	// 			return false;
	// 		}
	// 	}
	// 	if (s_previous == s) {
	// 		if (v_target > v) {
	// 			return false;
	// 		}
	// 		else {
	// 			return true;
	// 		}
	// 	}
	// }
	// else {
	// 	if (s_previous < s - test) {
	// 		if (s_now >= s - test) {
	// 			cout<<"true "<<snapshot.state<<endl;
	// 			return true;
	// 		}
	// 		else {
	// 			// cout<<"false"<<endl;
	// 			return false;
	// 		}
	// 	}
	// 	if (s_previous > s - test) {
	// 		if (s_now <= s - test) {
	// 			cout<<"true "<<snapshot.state<<endl;
	// 			return true;
	// 		}
	// 		else {
	// 			// cout<<"false"<<endl;
	// 			return false;
	// 		}
	// 	}
	// 	if (s_previous == s - test) {
	// 		if (v_target > v) {
	// 			return false;
	// 		}
	// 		else {
	// 			return true;
	// 		}
	// 	}
	// }
	
	throw invalid_argument("Value Error");
}

vector<double> unpack_snapshot(Vehicle::Snapshot snapshot) {
	Vehicle::Snapshot s = snapshot;
	return{s.lane, s.s, s.d, s.v, s.a};
}

map<int, vector<vector<double> > > filter_predictions_by_lane(map<int, vector<vector<double> > > predictions, int lane) {
	map<int, vector<vector<double> > > filtered;
	map<int, vector<vector<double> > >::iterator it = predictions.begin();
	vector<vector<vector<double> > > at_behind;
	while (it != predictions.end())
	{
		int v_id = it->first;
		vector<vector<double> > v = it->second;

		if (v[0][0] == lane && v_id != -1) {
			filtered[v_id] = v;
		}

		it++;
	}
	return filtered;
}