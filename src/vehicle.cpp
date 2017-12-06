#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include <limits>
#include <cassert>
#include "cost_functions.h"

Vehicle::Snapshot snap;

/**
* Initializes Vehicle
*/
Vehicle::Vehicle(double lane, double s, double d, double v, double a) {

	this->lane = lane;
	this->s = s;
	this->d = d;
	this->v = v;
	this->a = a;
	state = "CS";
	max_acceleration = -1;
	b = 0;
}

Vehicle::~Vehicle() {}

// TODO - Implement this method.
void Vehicle::update_state(map<int, vector < vector<double> > > predictions) {
	/*
	Updates the "state" of the vehicle by assigning one of the
	following values to 'self.state':

	"KL" - Keep Lane
	- The vehicle will attempt to drive its target speed, unless there is
	traffic in front of it, in which case it will slow down.

	"LCL" or "LCR" - Lane Change Left / Right
	- The vehicle will IMMEDIATELY change lanes and then follow longitudinal
	behavior for the "KL" state in the new lane.

	"PLCL" or "PLCR" - Prepare for Lane Change Left / Right
	- The vehicle will find the nearest vehicle in the adjacent lane which is
	BEHIND itself and will adjust speed to try to get behind that vehicle.

	INPUTS
	- predictions
	A dictionary. The keys are ids of other vehicles and the values are arrays
	where each entry corresponds to the vehicle's predicted location at the
	corresponding timestep. The FIRST element in the array gives the vehicle's
	current position. Example (showing a car with id 3 moving at 2 m/s):

	{
	3 : [
	{"s" : 4, "lane": 0},
	{"s" : 6, "lane": 0},
	{"s" : 8, "lane": 0},
	{"s" : 10, "lane": 0},
	]
	}

	*/
	// state = "KL"; // this is an example of how you change state.
	/*if (this->b != 2)
	{
		state = "LCR";
		this->b += 1;
	}
	else
	{
		state = "KL";
	}*/

	string state(get_next_state(predictions));

	this->state = state;
}

string Vehicle::get_next_state(map<int, vector<vector<double> > > predictions) {
	vector<string> states{ "KL", "LCL", "LCR" };
	// vector<string> states{ "KL" };
	if (this->lane == 0) {
		vector<string>::iterator result = find(states.begin(), states.end(), "LCL");
		if (result == states.end()) {
			cout << "That state is not in there!" << endl;
		}
		else {
			states.erase(result);
		}
	}
	if (this->lane == (lanes_available - 1)) {
		vector<string>::iterator result = find(states.begin(), states.end(), "LCR");
		if (result == states.end()) {
			cout << "That state is not in there!" << endl;
		}
		else {
			states.erase(result);
		}
	}

	// cout<<"\n Lane "<<this->lane<<endl;
	// cout<<"states "<<states.size()<<endl;
	// for(int i = 0;i<states.size();i++) {
	// 	cout<<"states "<<states[i]<<endl;
	// }

	vector<vector<string>> costs{};
	for (size_t i = 0; i < states.size(); i++) {
		// cout<<"state "<<states[i]<<endl;
		map<int, vector<vector<double> > > predictions_copy(predictions);
		vector<Vehicle::Snapshot> trajectory = trajectory_for_state(states[i], predictions_copy, 5);
		/*double cost(calculate_cost(trajectory, predictions));*/
		double cost(calculate_cost(*this, trajectory, predictions, true));
		string st = states[i];
		string co = to_string(cost);
		costs.push_back({states[i], to_string(cost)});
	}
	double max_cost = numeric_limits<double>::infinity();
	int indice;
	for (size_t i = 0; i < costs.size(); i++) {
		if (stoi(costs[i][1]) < max_cost) {
			max_cost = stoi(costs[i][1]);
			indice = i;
		}
	}
	return costs[indice][0];
}

vector<Vehicle::Snapshot> Vehicle::trajectory_for_state(string state, map<int, vector<vector<double> > > predictions, size_t horizon = 5) {
	Vehicle::Snapshot snappysnap = snapshot();
	this->state = state;
	vector<Vehicle::Snapshot> trajectory = {};
	trajectory.push_back(snappysnap);

	for (size_t i = 0; i < horizon; i++) {
		// restore_state_from_snapshot(snappysnap);
		// this->state = state;
		realize_state(predictions);
		//assert(0 <= this->lane < lanes_available && "lane" && this->lane);
		increment(1);
		trajectory.push_back(snapshot());
		map<int, vector<vector<double> > >::iterator it = predictions.begin();
		vector<vector<vector<double> > > at_behind;
		while (it != predictions.end())
		{
			int v_id = it->first;
			vector<vector<double> > v = it->second;

			it->second.erase(it->second.begin());
			it++;
		}
		this->state = "KL";
	}
	restore_state_from_snapshot(snappysnap);
	// for(int i = 0;i<trajectory.size();i++){
	// 	cout<<"trajectory "<<i<<" "<<trajectory[i].lane<<endl;
	// 	cout<<"trajectory "<<i<<" "<<trajectory[i].s<<endl;
	// 	cout<<"trajectory "<<i<<" "<<trajectory[i].d<<endl;
	// 	cout<<"trajectory "<<i<<" "<<trajectory[i].v<<endl;
	// 	cout<<"trajectory "<<i<<" "<<trajectory[i].a<<endl;
	// 	cout<<"trajectory "<<i<<" "<<trajectory[i].state<<endl;
	// }
	return trajectory;
}

Vehicle::Snapshot Vehicle::snapshot() {
	snap.lane = this->lane;
	snap.s = this->s;
	snap.d = this->d;
	snap.v = this->v;
	snap.a = this->a;
	snap.state = this->state;
	return snap;
}

void Vehicle::restore_state_from_snapshot(Vehicle::Snapshot snapshot) {
	Vehicle::Snapshot s = snapshot;
	this->lane = s.lane;
	this->s = s.s;
	this->d = s.d;
	this->v = s.v;
	this->a = s.a;
	this->state = s.state;
}

void Vehicle::configure(vector<double> road_data) {
	/*
	Called by simulator before simulation begins. Sets various
	parameters which will impact the ego vehicle.
	*/
	target_speed = road_data[0];
	lanes_available = road_data[1];
	goal_s = road_data[2];
	goal_lane = road_data[3];
	max_acceleration = road_data[4];
}

string Vehicle::display() {

	ostringstream oss;

	oss << "s:    " << this->s << "\n";
	oss << "d:    " << this->d << "\n";
	oss << "lane: " << this->lane << "\n";
	oss << "v:    " << this->v << "\n";
	oss << "a:    " << this->a << "\n";

	return oss.str();
}

void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
	this->v += this->a * dt;
}

vector<double> Vehicle::state_at(int t) {

	/*
	Predicts state of vehicle in t seconds (assuming constant acceleration)
	*/
	double s = this->s + this->v * t + this->a * t * t / 2;
	double v = this->v + this->a * t;
	return{ this->lane, s, this->d, v, this->a };
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

	/*
	Simple collision detection.
	*/
	vector<double> check1 = state_at(at_time);
	vector<double> check2 = other.state_at(at_time);
	return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1;

	for (int t = 0; t < timesteps + 1; t++)
	{
		if (collides_with(other, t))
		{
			collider_temp.collision = true;
			collider_temp.time = t;
			return collider_temp;
		}
	}

	return collider_temp;
}

void Vehicle::realize_state(map<int, vector < vector<double> > > predictions) {

	/*
	Given a state, realize it by adjusting acceleration and lane.
	Note - lane changes happen instantaneously.
	*/
	string state = this->state;
	if (state.compare("CS") == 0)
	{
		realize_constant_speed();
	}
	else if (state.compare("KL") == 0)
	{
		realize_keep_lane(predictions);
	}
	else if (state.compare("LCL") == 0)
	{
		realize_lane_change(predictions, "L");
	}
	else if (state.compare("LCR") == 0)
	{
		realize_lane_change(predictions, "R");
	}
	else if (state.compare("PLCL") == 0)
	{
		realize_prep_lane_change(predictions, "L");
	}
	else if (state.compare("PLCR") == 0)
	{
		realize_prep_lane_change(predictions, "R");
	}

}

void Vehicle::realize_constant_speed() {
	a = 0;
}

double Vehicle::_max_accel_for_lane(map<int, vector<vector<double> > > predictions, int lane, double s) {

	double delta_v_til_target = target_speed - v;
	double max_acc = min(max_acceleration, delta_v_til_target);

	map<int, vector<vector<double> > >::iterator it = predictions.begin();
	vector<vector<vector<double> > > in_front;
	while (it != predictions.end())
	{

		int v_id = it->first;

		vector<vector<double> > v = it->second;

		if ((v[0][0] == lane) && (v[0][1] > s))
		{
			in_front.push_back(v);

		}
		it++;
	}

	if (in_front.size() > 0)
	{
		int min_s = 1000;
		vector<vector<double>> leading = {};
		for (size_t i = 0; i < in_front.size(); i++)
		{
			if ((in_front[i][0][1] - s) < min_s)
			{
				min_s = (in_front[i][0][1] - s);
				leading = in_front[i];
			}
		}

		double next_pos = leading[1][1];
		double my_next = s + this->v;
		double separation_next = next_pos - my_next;
		double available_room = separation_next - preferred_buffer;
		available_room = max(-2.0, available_room);
		max_acc = min(max_acc, available_room);
	}

	return max_acc;

}

void Vehicle::realize_keep_lane(map<int, vector< vector<double> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int, vector< vector<double> > > predictions, string direction) {
	int delta = 1;
	if (direction.compare("L") == 0)
	{
		delta = -1;
	}
	this->lane += delta;
	double lane = this->lane;
	double s = this->s;
	this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int, vector<vector<double> > > predictions, string direction) {
	int delta = 1;
	if (direction.compare("L") == 0)
	{
		delta = -1;
	}
	int lane = this->lane + delta;

	map<int, vector<vector<double> > >::iterator it = predictions.begin();
	vector<vector<vector<double> > > at_behind;
	while (it != predictions.end())
	{
		int v_id = it->first;
		vector<vector<double> > v = it->second;

		if ((v[0][0] == lane) && (v[0][1] <= this->s))
		{
			at_behind.push_back(v);

		}
		it++;
	}
	if (at_behind.size() > 0)
	{

		int max_s = -1000;
		vector<vector<double> > nearest_behind = {};
		for (size_t i = 0; i < at_behind.size(); i++)
		{
			if ((at_behind[i][0][1]) > max_s)
			{
				max_s = at_behind[i][0][1];
				nearest_behind = at_behind[i];
			}
		}
		double target_vel = nearest_behind[1][1] - nearest_behind[0][1];
		double delta_v = this->v - target_vel;
		double delta_s = this->s - nearest_behind[0][1];
		if (delta_v != 0)
		{

			double time = -2 * delta_s / delta_v;
			double a;
			if (time == 0)
			{
				a = this->a;
			}
			else
			{
				a = delta_v / time;
			}
			if (a > this->max_acceleration)
			{
				a = this->max_acceleration;
			}
			if (a < -this->max_acceleration)
			{
				a = -this->max_acceleration;
			}
			this->a = a;
		}
		else
		{
			double my_min_acc = max(-this->max_acceleration, -delta_s);
			this->a = my_min_acc;
		}

	}

}

vector<vector<double> > Vehicle::generate_predictions(int horizon = 10) {

	vector<vector<double> > predictions;
	for (int i = 0; i < horizon; i++)
	{
		vector<double> check1 = state_at(i);
		vector<double> lane_s = { check1[0], check1[1], check1[2] };
		predictions.push_back(lane_s);
	}
	return predictions;

}