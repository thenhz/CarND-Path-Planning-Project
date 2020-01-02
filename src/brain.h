#ifndef BRAIN_H
#define BRAIN_H

#include <vector>
#include <map>
#include <string>
#include <vector>
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::map;
using std::string;
using std::vector;

class Brain
{
public:
    // Constructors
    Brain();
    Brain(nlohmann::json telemetry, int lane, string state = "CS");

    // Destructor
    virtual ~Brain();

    struct driving_params
    {
        int lane;
        double vel;
        double car_s;
        double acc;
        double yaw;
        string new_state;
        bool proposing_change_lane;
    };

    struct trajectory
    {
        vector<double> next_x_vals;
        vector<double> next_y_vals;
    };

    map<int, string> plcx_labels = {{-1,"PLCL"}, {1, "PLCR"}};
    map<int, string> lcx_labels = {{-1,"LCL"}, {1, "LCR"}};

    double car_x, car_y, car_s, car_d, car_yaw, car_speed, end_path_s, end_path_d;
    string state;
    int lane, prev_size, preferred_buffer = 6, lanes_available = 3;
    nlohmann::json previous_path_x, previous_path_y, sensor_fusion;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    void SetLane(int lane);

    void SetTelemetry(nlohmann::json telemetry);

    void setMaps(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);

    driving_params choose_next_state();

    driving_params generate_trajectory(string state);

    trajectory generate_spline(double initial_x, double initial_y, double initial_yaw, int new_lane, double new_s, double vel);

    driving_params constant_speed_trajectory();
    driving_params keep_lane_trajectory();
    driving_params lane_change_trajectory(int lane_direction);
    driving_params prep_lane_change_trajectory(int lane_direction);

    bool get_vehicle_ahead(Brain::driving_params *rVehicle, int lane, int allowed_distance);

    bool get_vehicle_behind(Brain::driving_params *rVehicle, int lane, int allowed_distance);

    vector<float> get_kinematics(int _lane, int allowed_distance);

    driving_params generateDrivingParams(int _lane, double allowed_distance, string _state);

    vector<string> successor_states();

    void realize_next_state(Brain::driving_params trajectory);
};

#endif