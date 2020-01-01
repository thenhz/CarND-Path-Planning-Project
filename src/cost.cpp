#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "brain.h"

using std::string;
using std::vector;

const float SWITCH_LANE = pow(10, 6);
const float GO_FAST = pow(10, 6);


float switch_lane(const Brain::driving_params params, const int current_lane) {
  return 0.5;
};

float go_fast(const Brain::driving_params params, const int current_lane) {
  return 0.5;
};

float calculate_cost(const Brain &brain, 
                     const Brain::driving_params params) {
  // Sum weighted cost functions to get total cost for trajectory.
    /*
  float cost = 0.0;

  // Add additional cost functions here.
  vector<std::function<float(const Brain::driving_params params, const int current_lane)>> cf_list = {switch_lane, go_fast};
  vector<float> weight_list = {SWITCH_LANE, GO_FAST};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](brain, params);
    cost += new_cost;
  }

  return cost;*/
  return 0.5;
};