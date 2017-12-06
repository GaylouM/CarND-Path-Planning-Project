#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "cost_functions.h"
#include <typeinfo>

using namespace std;

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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector< vector<double> > build_path(double car_x, double car_y, double car_s, double car_yaw, vector<double> previous_path_x, vector<double> previous_path_y, int lane, double ref_val, int prev_size, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s) {
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  if(prev_size < 2) {
    double prev_car_x = car_x - cos(ref_yaw);
    double prev_car_y = car_y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];

    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  // cout<<"car_s "<<car_s<<endl;
  vector<double> next_wp0 = getXY(car_s+30+10,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60+10,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90+10,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for(int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
  }

  // for(int i = 0;i<ptsx.size();i++) {
  //   cout<<"ptsx "<<i<<" "<<ptsx[i]<<endl;
  //   cout<<"ptsy "<<i<<" "<<ptsy[i]<<endl;
  // }

  tk::spline s;

  s.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  for(int i = 0; i < previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);

  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
  double x_add_on = 0;

  for(int i = 1; i <= 50-previous_path_x.size(); i++) {
    double N = (target_dist/(.02*ref_val/2.24));
    double x_point = x_add_on+(target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  return {next_x_vals, next_y_vals};
}

int get_lane(double d) {
  int l;
  if(((d-4)/4) > 1 && ((d-4)/4) <= 2) {
    l = 2;
  }
  else if(((d-4)/4) > 0 && ((d-4)/4) <= 1) {
    l = 1;
  }
  else if(((d-4)/4) >= -1 && ((d-4)/4) <= 0) {
    l = 0;
  }
  return l;
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

  int lane = 1;

  double ref_val = 0.0;

  map<int, Vehicle> vehicles;

  int counter = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_val,&vehicles,&counter,&max_s](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    counter += 1;

    // cout <<"\n"<< "STEP N°" << counter <<"\n"<< endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      // cout <<"test"<< endl;

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          // cout << "car_s1 " <<car_s<< endl;
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
      		// cout << "car_x " << car_x << endl;
      		// cout << "car_y " << car_y << endl;
      		// cout << "car_s " << car_s << endl;
      		// cout << "car_d " << car_d << endl;
      		// cout << "car_yaw " << car_yaw << endl;
      		// cout << "car_speed " << car_speed << endl;
      		// cout << "previous_path_x " << previous_path_x << endl;
      		// cout << "previous_path_y " << previous_path_y << endl;
      		// cout << "end_path_s " << end_path_s << endl;
      		// cout << "end_path_d " << end_path_d << endl;
      		// cout << "sensor_fusion[0][0] " << sensor_fusion[0][0] << endl;
      		// cout << "sensor_fusion[0][1] " << sensor_fusion[0][1] << endl;
      		// cout << "sensor_fusion[0][2] " << sensor_fusion[0][2] << endl;
      		// cout << "sensor_fusion[0][3] " << sensor_fusion[0][3] << endl;
      		// cout << "sensor_fusion[0][4] " << sensor_fusion[0][4] << endl;
      		// cout << "sensor_fusion[0][5] " << sensor_fusion[0][5] << endl;
      		// cout << "sensor_fusion[0][6] " << sensor_fusion[0][6] << endl;

          // for (int i = 0; i < sensor_fusion.size(); i++) {
          //   cout << "Vehicle n°" <<i<<": "<< endl;
          //   cout << "          x: "<< sensor_fusion[i][1] << endl;
          //   cout << "          y: "<< sensor_fusion[i][2] << endl;
          //   cout << "          vx: "<< sensor_fusion[i][3] << endl;
          //   cout << "          vy: "<< sensor_fusion[i][4] << endl;
          //   cout << "          s: "<< sensor_fusion[i][5] << endl;
          //   cout << "          d: "<< sensor_fusion[i][6] << endl;
          //  }

          // cout << "car_s_1_b_all " <<car_s<< endl;

          int prev_size = previous_path_x.size();

          // cout <<"prev_size "<<prev_size<< endl;

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          if(counter < 300) {
      
           bool too_close = false;
      
           for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            if (d<(2 + 4 * lane + 2) && d>(2 + 4 * lane - 2)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
      
              check_car_s+=((double)prev_size*.02*check_speed);
      
              if((check_car_s > car_s) && ((check_car_s-car_s) < 30)) {
                too_close = true;
                if(lane > 0) {
                  lane = 0;
                }
                else if(lane == 0) {
                  lane = 1;
                }
              }
            }
           }
      
            if(too_close) {
              ref_val -= .224;
            }
            else if(ref_val < 49.5) {
              ref_val += .224;
            }
            // cout << "car_speed " <<car_speed<< endl;
          }

          // cout << "car_s2 " <<car_s<< endl;

          if(counter >= 300) {

          	// cout << "car_speed " <<car_speed<< endl;

            map<int, Vehicle>::iterator it;
            double pos_closest;
            int id;

            for (int i = 0; i < sensor_fusion.size(); i++) {

              double d = sensor_fusion[i][6];
              double s = sensor_fusion[i][5];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double v = sqrt(vx*vx+vy*vy);

              if(d >= 0 && d <= 12) {
                it = vehicles.find((int)sensor_fusion[i][0]+1);
                if (it == vehicles.end()){
                  Vehicle vehicle = Vehicle(get_lane(d), s, d, v, 0);
                  vehicles.insert(std::pair<int, Vehicle>((int)sensor_fusion[i][0]+1, vehicle));
                }
                else {
                  it->second.lane = get_lane(d);
                  it->second.s = s;
                  it->second.d = d;
                  it->second.v = v;
                 //  if(it->second.s < car_s-5 && it->second.s > car_s-25) {
                 //  	cout << "\n VEHICLES " <<it->first<< endl;
	                // // cout << "    LANE:" <<it->second.lane<< endl;
	                // // cout << "       S:" <<it->second.s<< endl;
	                // // cout << "       V:" <<it->second.v<< endl;
	                // // cout << "   STATE:" <<it->second.state<< endl;
                 //  }
                  
                }
                double closest_front = numeric_limits<double>::infinity();
                double diff = car_s - s;
              	if(get_lane(d) == 0) {
                	if(car_s < s && diff < closest_front) {
                  		closest_front = diff;
                  		pos_closest = s;
                  		id = it->first;
                	}
              	}
              }
            }

            // cout <<"Vehicles "<<id<< endl;
            // cout <<"pos_closest "<<pos_closest<< endl;
            // cout <<"car_s "<<car_s<< endl;
            // cout <<"pos_closest-car_s "<<pos_closest-car_s<< endl;

            it = vehicles.find(-1);
            if (it == vehicles.end()){
              vector<double> ego_config = { 49.9*0.44704,3,2*max_s,2.0,0.15 };
              Vehicle ego = Vehicle(get_lane(car_d), car_s, car_d, car_speed*0.44704, 0.0);
              // cout << "car_s_a_ego " <<car_s<< endl;
              ego.configure(ego_config);
              // ego.state = "KL";
              vehicles.insert(std::pair<int, Vehicle>(-1, ego));
            }
            else {
              it->second.s = car_s;
              it->second.d = car_d;
              // cout <<"it->second.v"<<it->second.v<< endl;
              // it->second.v = car_speed*0.44704;
            }

            // cout <<"test"<< endl;

            map<int, vector<vector<double>>> predictions;

            // cout << "car_s_b_generate " <<car_s<< endl;

            it = vehicles.begin();
            while (it != vehicles.end())
            {
              int v_id = it->first;
              // cout<<"vitesse vehicule "<<v_id<<" "<<it->second.v<<endl;
              vector<vector<double> > preds = it->second.generate_predictions(10);
              predictions[v_id] = preds;
              it++;
            }
            // cout << "car_s_2_a_generate " <<car_s<< endl;

            it = vehicles.begin();
            while (it != vehicles.end())
            {
              int v_id = it->first;
              // cout <<"test "<<v_id<< endl;
              if (v_id == -1)
              {
                // cout << "car_s_b_update_state " <<car_s<< endl;
                it->second.update_state(predictions);
                // cout << "car_s_b_realize_state " <<car_s<< endl;
                // cout << "it->second.lane " <<it->second.lane<< endl;
                it->second.realize_state(predictions);
                // cout << "it->second.lane " <<it->second.lane<< endl;
                // cout << "car_s_2_a_realize_state " <<car_s<< endl;
                ref_val = it->second.v/0.44704;
                it->second.v = it->second.v + it->second.a;
                // cout << "ref_val " <<ref_val<< endl;
                lane = it->second.lane;
                // else {
                //   it->second.v = ref_val;
                //   it->second.lane = lane;
                // }

                // lane = it->second.lane;

                // cout << "s " <<it->second.s<< endl;
                // cout << "v " <<it->second.v<< endl;
                // cout << "a " <<it->second.a<< endl;

                // cout << "new_v " <<it->second.v<< endl;
                // cout << "lane " <<it->second.lane<< endl;
                // cout << "state " <<it->second.state<< endl;
                // cout << "car_s_3 " <<car_s<< endl;
              }

              // ref_val = it->second.v;
              // it->second.v = it->second.v + it->second.a;
              // lane = it->second.lane;

              it++;
            }
        }																		

            // cout << "lane " <<lane<< endl;
            // cout << "ref_val " <<ref_val<< endl;

            // cout << "car_s3 " <<car_s<< endl;

        	// vehicles.clear();

            vector<vector<double>> next_xy_vals;

            next_xy_vals = build_path(car_x, car_y, car_s, car_yaw, previous_path_x, previous_path_y, lane, ref_val, prev_size, map_waypoints_x, map_waypoints_y, map_waypoints_s);

          	json msgJson;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_xy_vals[0];
          	msgJson["next_y"] = next_xy_vals[1];

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
