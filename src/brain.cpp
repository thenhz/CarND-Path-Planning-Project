#include "brain.h"
#include "spline.h"
#include "cost.h"
#include "helpers.h"
#include <iostream>
#include "DrivingParams.h"

using nlohmann::json;
using std::map;
using std::string;
using std::vector;

Brain::Brain()
{
    this->state = "CS";
    this->car_speed = 0.0;
    this->lanes_available = 3;
};

Brain::~Brain() {}

void Brain::SetLane(int lane)
{
    this->lane = lane;
};

void Brain::SetTelemetry(nlohmann::json j)
{
    // Main car's localization Data
    
    this->car_x = j[1]["x"];
    this->car_y = j[1]["y"];
    //check_car_s += ((double)prev_size * .02 * check_speed);
    //this->car_d = j[1]["d"];
    this->car_yaw = j[1]["yaw"];
    //this->car_speed = j[1]["speed"];

    // Previous path data given to the Planner
    this->previous_path_x = j[1]["previous_path_x"];
    this->previous_path_y = j[1]["previous_path_y"];
    // Previous path's end s and d values
    this->end_path_s = j[1]["end_path_s"];
    this->end_path_d = j[1]["end_path_d"];
    // Sensor Fusion Data, a list of all other cars on the same side
    //   of the road.
    this->sensor_fusion = j[1]["sensor_fusion"];

    this->prev_size = previous_path_x.size();
    //float env_car_speed = j[1]["speed"];
    this->car_s = (float)j[1]["s"];// + ((float)this->prev_size * .02 * env_car_speed);
    if (this->prev_size > 0)
    {
        this->car_s = this->end_path_s;
    }
};

void Brain::setMaps(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy)
{
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
    this->map_waypoints_dx = map_waypoints_dx;
    this->map_waypoints_dy = map_waypoints_dy;
};

DrivingParams Brain::choose_next_state()
{
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<DrivingParams> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
    {
        DrivingParams trajectory = generate_trajectory(*it);
        if (trajectory.lane >= 0)
        {
            cost = calculate_cost(*this, &trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
            std::cout << trajectory.new_state << " cost:" << cost << " ";
        }
    }
    std::cout << std::endl;
    //std::cout << "number of proposed trajectories:" << final_trajectories.size();
    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    auto _trajectory = final_trajectories[best_idx];
    //std::cout << "State:" << state << "; vel:"<< _trajectory.vel << "; car_s:"<< _trajectory.car_s << "; lane:"<< _trajectory.lane<< std::endl;
    std::cout << " - Selected state: " << _trajectory.new_state << std::endl;
    return _trajectory;
}

DrivingParams Brain::generate_trajectory(string state)
{
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    DrivingParams trajectory = DrivingParams();
    if (state.compare("CS") == 0)
    {
        trajectory = constant_speed_trajectory();
    }
    else if (state.compare("KL") == 0)
    {
        trajectory = keep_lane_trajectory();
    }
    else if (state.compare("LCL") == 0)
    {
        trajectory = lane_change_trajectory(-1);
    }
    else if (state.compare("LCR") == 0)
    {
        trajectory = lane_change_trajectory(1);
    }
    else if (state.compare("PLCL") == 0)
    {
        trajectory = prep_lane_change_trajectory(-1);
    }
    else if (state.compare("PLCR") == 0)
    {
        trajectory = prep_lane_change_trajectory(1);
    }

    return trajectory;
};

vector<string> Brain::successor_states()
{
    // Provides the possible next states given the current state for the FSM
    //   discussed in the course, with the exception that lane changes happen
    //   instantaneously, so LCL and LCR can only transition back to KL.
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    //std::cout << "CURRENT_STATE: " << state << std::endl;
    if (state.compare("KL") == 0)
    {
        states.push_back("PLCL");
        states.push_back("PLCR");
    }
    else if (state.compare("PLCL") == 0)
    {
        if (lane != 0)
        {
            states.push_back("PLCL");
            states.push_back("LCL");
            //std::cout << " Evaluating LCL" << std::endl;
        }
    }
    else if (state.compare("PLCR") == 0)
    {
        if (lane != lanes_available - 1)
        {
            states.push_back("PLCR");
            states.push_back("LCR");
            //std::cout << " Evaluating LCR" << std::endl;
        }
    }

    // If state is "LCL" or "LCR", then just return "KL"
    return states;
};

DrivingParams Brain::constant_speed_trajectory()
{
    //std::cout << "constant_speed_trajectory" << std::endl;
    DrivingParams params = DrivingParams();

    params.car_s = this->car_s;
    params.lane = this->lane;
    params.vel = this->car_speed;
    params.new_state = "CS";
    params.proposed_lane = this->lane;

    return params;
};

DrivingParams Brain::keep_lane_trajectory()
{
    //std::cout << "keep_lane_trajectory" << std::endl;
    return this->generateDrivingParams(this->lane, 30.0, "KL");
};

DrivingParams Brain::generateDrivingParams(int _lane, double allowed_distance, string _state)
{

    vector<float> kinematics = get_kinematics(_lane, allowed_distance);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];

    DrivingParams params = DrivingParams();

    params.car_s = new_s;
    params.lane = _lane;
    params.vel = new_v;
    params.new_state = _state;
    params.proposed_lane = lane;
    params.proposed_vel = new_v;

    return params;
};

DrivingParams Brain::lane_change_trajectory(int lane_direction)
{
    // Generate a lane change trajectory.
    int new_lane = this->lane + lane_direction;
    DrivingParams trajectory = DrivingParams();

    if (new_lane < 0 || new_lane > lanes_available - 1)
    {
        trajectory.lane = -1;
        return trajectory;
    }

    DrivingParams vehicle_ahead = DrivingParams();
    DrivingParams vehicle_behind = DrivingParams();

    //TODO: fix this
    /*
    if (get_vehicle_ahead(&vehicle_ahead, new_lane, 25.0))
    {
        //NULL if lane is busy
        return trajectory;
    }*/
    //std::cout << "creating lane_change_trajectory params!!!!!" << std::endl;
    trajectory = this->generateDrivingParams(new_lane, 25.0, lcx_labels[lane_direction]);
    trajectory.vel += 0.224;
    trajectory.proposed_lane = new_lane;
    return trajectory;
};

DrivingParams Brain::prep_lane_change_trajectory(int lane_direction)
{

    // Generate a trajectory preparing for a lane change.
    int new_lane = this->lane + lane_direction;
    DrivingParams trajectory = DrivingParams();
    DrivingParams vehicle_behind = DrivingParams(), current_lane_kinematics = DrivingParams(), proposed_trajectory = DrivingParams();

    //current_lane_kinematics = this->generateDrivingParams(this->lane, 30.0, plcx_labels[lane_direction]);
    
    if (new_lane < 0 || new_lane > lanes_available - 1)
    {
        current_lane_kinematics.lane = -1;
        return current_lane_kinematics;
    }
    else
    {
        //std::cout << "PREPARE SUGGESTING LANE CHANGE" << std::endl;
        DrivingParams vehicle_ahead = DrivingParams();
        if (!get_vehicle_ahead(&vehicle_ahead, new_lane, 15.0))
        {
            proposed_trajectory = generateDrivingParams(new_lane, 15.0, plcx_labels[lane_direction]);
            trajectory = generateDrivingParams(this->lane, 30.0, plcx_labels[lane_direction]);
            trajectory.proposed_lane = proposed_trajectory.lane;
            trajectory.proposed_vel = proposed_trajectory.vel ;
        }
        else
        {
            trajectory.lane = -1;
        }
        return trajectory;
    }
    /*
    //TODO: check if vehicle_nehind is faster
    if (!get_vehicle_behind(&vehicle_behind, this->lane, 25.0))
    {
        DrivingParams new_lane_kinematics = this->generateDrivingParams(new_lane, 25.0,plcx_labels[lane_direction]);
        if (current_lane_kinematics.vel < new_lane_kinematics.vel)
        {
            std::cout << "SUGGESTING LANE CHANGE" << std::endl;
            return new_lane_kinematics;
        }
        else
        {
            return current_lane_kinematics;
        }
    }
    else
    {
        return current_lane_kinematics;
    }*/
};

vector<float> Brain::get_kinematics(int _lane, int allowed_distance)
{
    // Gets next timestep kinematics (position, velocity, acceleration)
    //   for a given lane. Tries to choose the maximum velocity and acceleration,
    //   given other vehicle positions and accel/velocity constraints.
    float new_position;
    float new_velocity;
    float new_accel, acc_v_ahead;
    DrivingParams vehicle_ahead = DrivingParams();
    DrivingParams vehicle_behind = DrivingParams();

    if (get_vehicle_ahead(&vehicle_ahead, _lane, allowed_distance))
    {

        //new_velocity = this->car_speed - 0.224;
        //acc_v_ahead = std::min(0.224,(this->car_speed - vehicle_ahead.vel)/0.2);
        //new_velocity = (this->car_speed - acc_v_ahead < vehicle_ahead.vel) ? vehicle_ahead.vel : this->car_speed - acc_v_ahead;
        //new_velocity =  this->car_speed - 0.224;
        new_velocity = this->car_speed - std::min(0.224,(this->car_speed - vehicle_ahead.vel)*0.02);
    }
    else
    {
        new_velocity = (this->car_speed + 0.224 > 49.5) ? 49.5 : this->car_speed + 0.224;
    }

    new_accel = (new_velocity - this->car_speed) / 0.2; // Equation: (v_1 - v_0)/t = acceleration
    new_position = this->car_s + new_velocity + new_accel / 2.0;

    return {new_position, new_velocity, new_accel};
};

bool Brain::get_vehicle_ahead(DrivingParams *rVehicle, int _lane, int allowed_distance)
{
    // Returns a true if a vehicle is found ahead of the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.

    bool found_vehicle = false;
    //Sensor fusion part
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float d = sensor_fusion[i][6];
        if (d < (2 + 4 * _lane + 2) && d > (2 + 4 * _lane - 2))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)prev_size * .02 * check_speed);
            if ((check_car_s > car_s) && ((check_car_s - car_s) < allowed_distance))
            {
                found_vehicle = true;
                rVehicle->car_s = check_car_s;
                rVehicle->lane = _lane;
                rVehicle->vel = check_speed;
                //std::cout << std::endl << _lane <<" VEHICLE AHEAD m." << allowed_distance << " !! s:" << check_car_s << "; lane:" << _lane << "; vel:" << check_speed << std::endl;
            }
        }
    }
    return found_vehicle;
};

bool Brain::get_vehicle_behind(DrivingParams *rVehicle, int _lane, int allowed_distance)
{
    // Returns a true if a vehicle is found ahead of the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.

    bool found_vehicle = false;
    //Sensor fusion part
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float d = sensor_fusion[i][6];
        if (d < (2 + 4 * _lane + 2) && d > (2 + 4 * _lane - 2))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)prev_size * .02 * check_speed);
            if ((check_car_s < car_s) && ((car_s - check_car_s) < allowed_distance))
            {
                found_vehicle = true;
                rVehicle->car_s = check_car_s;
                rVehicle->lane = _lane;
                rVehicle->vel = check_speed;
                //std::cout << "VEHICLE BEHIND m." << allowed_distance << " !! s:" << check_car_s << "; lane:" << _lane << "; vel:" << check_speed << std::endl;
            }
        }
    }
    return found_vehicle;
};

Brain::trajectory Brain::generate_spline(double initial_x, double initial_y, double initial_yaw, int new_lane, double new_s, double vel)
{
    //spaced waypoints to calculate spline
    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = initial_x;
    double ref_y = initial_y;
    double ref_yaw = deg2rad(initial_yaw);

    //if we don't have enough points, use the car as starting ref
    if (prev_size < 2)
    {

        double prev_car_x = ref_x - cos(ref_yaw);
        double prev_car_y = ref_y - sin(ref_yaw);
        ptsx.push_back(prev_car_x);
        ptsx.push_back(ref_x);
        ptsy.push_back(prev_car_y);
        ptsy.push_back(ref_y);
    }
    else
    {
        //use previous path returned by simulator
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];
        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * new_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * new_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * new_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++)
    {
        // shift car reference angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }
    /*for(auto i: ptsx){
        std::cout << i << " " << std::endl;
    }*/

    //create spline
    tk::spline s;
    s.set_points(ptsx, ptsy);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    //Trajectory starts from prevoius points not already walked
    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // calculate how to break up spline to travel at the desired velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

    double x_add_on = 0;
    for (int i = 1; i <= 50 - previous_path_x.size(); i++)
    {
        double N = (target_dist / (.02 * vel / 2.24));
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
    trajectory trj;
    trj.next_x_vals = next_x_vals;
    trj.next_y_vals = next_y_vals;
    return trj;
};

void Brain::realize_next_state(DrivingParams trajectory)
{
    // Sets state and kinematics for ego vehicle using the last state of the trajectory.
    //this->car_s = trajectory.car_s;
    this->lane = trajectory.lane;
    this->car_speed = trajectory.vel;
    this->state = trajectory.new_state;
    //TODO: set yaw accel et al?
};