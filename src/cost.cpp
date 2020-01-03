#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "brain.h"
#include <iostream>
#include "DrivingParams.h"

using std::string;
using std::vector;

const float SWITCH_LANE = pow(10, 5);
const float GO_FAST = pow(10, 6);

float switch_lane(const DrivingParams *params, const int current_lane, const double current_speed)
{
    //std::cout << "current_lane:" << current_lane << "proposed_change_lane:" << params->proposing_change_lane << "proposed_lane:" << params->lane << " current_speed:" << current_speed << std::endl; 
    //std::cout << "LANE:" <<params->lane << std::endl;
    bool switch_proposed = (params->proposed_lane != current_lane);
    float cost = 0.0;
    float cost_proposed_speed = 0.01* (abs(params->proposed_vel - current_speed) / current_speed) ;
    float cost_current_speed = abs(params->vel - current_speed) / current_speed;
    //float switch_lane = (switch_lane && (params->vel > current_speed )) ? 0.0 : 0.1;
    float hint = (params->new_state.compare("LCL")==0 || params->new_state.compare("LCR")==0) ? 0.0001 : 0.0; 
    return (switch_proposed) ? 0.001 + (cost_current_speed)-hint : 0.0;
    //return 1 - float(abs(params->proposed_lane - current_lane)) + switch_lane;
};

float go_fast(const DrivingParams *params, const int current_lane, const double current_speed)
{
    float cost = (2.0*49.5 - params->proposed_vel - current_speed)/49.5;
    return cost;
};

float calculate_cost(const Brain &brain,
                     const DrivingParams *params)
{
    // Sum weighted cost functions to get total cost for trajectory.

    float cost = 0.0;

    // Add additional cost functions here.
    vector<std::function<float(const DrivingParams *params, const int current_lane, const double current_speed)>> cf_list = {switch_lane, go_fast};
    vector<float> weight_list = {SWITCH_LANE, GO_FAST};

    for (int i = 0; i < cf_list.size(); ++i)
    {
        //TODO: fix weight
        float new_cost = /*weight_list[i] **/ cf_list[i](params, brain.lane, brain.car_speed);
        cost += new_cost;
    }

    return cost;
};