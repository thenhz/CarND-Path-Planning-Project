#ifndef COST_H
#define COST_H

#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "brain.h"


using std::map;
using std::string;
using std::vector;

float switch_lane(const Brain::driving_params, const int current_lane);

float go_fast(const Brain::driving_params, const int current_lane);

float calculate_cost(const Brain &brain, const Brain::driving_params params);

#endif  // COST_H