#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

	struct collider {

		bool collision; // is there a collision?
		int  time; // time collision happens

	};

	struct Snapshot {

		double lane;
		double  s;
		double  d;
		double  v;
		double  a;
		string  state;
	};

	int L = 1;

	int preferred_buffer = 6; // impacts "keep lane" behavior.

	double lane;

	double s;

	double d;

	double v;

	double a;

	double target_speed;

	double lanes_available;

	double max_acceleration;

	double goal_lane;

	double goal_s;

	string state;

	int b;

	/**
	* Constructor
	*/
	Vehicle(double lane, double s, double d, double v, double a);

	/**
	* Destructor
	*/
	virtual ~Vehicle();

	void update_state(map<int, vector <vector<double> > > predictions);

	void configure(vector<double> road_data);

	string display();

	void increment(int dt);

	vector<double> state_at(int t);

	bool collides_with(Vehicle other, int at_time);

	collider will_collide_with(Vehicle other, int timesteps);

	void realize_state(map<int, vector < vector<double> > > predictions);

	void realize_constant_speed();

	double _max_accel_for_lane(map<int, vector<vector<double> > > predictions, int lane, double s);

	void realize_keep_lane(map<int, vector< vector<double> > > predictions);

	void realize_lane_change(map<int, vector< vector<double> > > predictions, string direction);

	void realize_prep_lane_change(map<int, vector< vector<double> > > predictions, string direction);

	vector<vector<double> > generate_predictions(int horizon);

	Snapshot snapshot();

	void restore_state_from_snapshot(Vehicle::Snapshot snapshot);

	vector<Snapshot> trajectory_for_state(string state, map<int, vector<vector<double> > > predictions, size_t horizon);

private:

	string get_next_state(map<int, vector<vector<double> > > predictions);

	/*vector<Snapshot> trajectory_for_state(string state, map<int, vector<vector<int> > > predictions, int horizon = 5);*/

};

#endif