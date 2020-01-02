#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "brain.h"
#include <iostream>

using std::string;
using std::vector;

const float SWITCH_LANE = pow(10, 6);
const float GO_FAST = pow(10, 6);

float switch_lane(const Brain::driving_params params, const int current_lane, const double current_speed)
{
    std::cout << "current_lane:" << current_lane << "proposed_change_lane:" << params.proposing_change_lane << "proposed_lane:" << params.lane << " current_speed:" << current_speed << std::endl; 
    float cost = 0.0;
    if (params.proposing_change_lane) {
        cost = 0.5;
        if(params.lane!=current_lane){
            cost= 0.3;
        }
    } else {
        cost = 1;
    }
    return cost;
};

float go_fast(const Brain::driving_params params, const int current_lane, const double current_speed)
{
    float cost = (2.0*49.5 - params.vel - current_speed)/49.5;
    return cost;
};

float calculate_cost(const Brain &brain,
                     const Brain::driving_params params)
{
    // Sum weighted cost functions to get total cost for trajectory.

    float cost = 0.0;

    // Add additional cost functions here.
    vector<std::function<float(const Brain::driving_params params, const int current_lane, const double current_speed)>> cf_list = {switch_lane, go_fast};
    vector<float> weight_list = {SWITCH_LANE, GO_FAST};

    for (int i = 0; i < cf_list.size(); ++i)
    {
        //TODO: fix weight
        float new_cost = /*weight_list[i] **/ cf_list[i](params, brain.lane, brain.car_speed);
        cost += new_cost;
    }

    return cost;
};