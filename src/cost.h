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

float switch_lane(const DrivingParams *params, const int current_lane, const double current_speed);

float go_fast(const DrivingParams *params, const int current_lane, const double current_speed);

float calculate_cost(const Brain &brain, const DrivingParams *params);

#endif  // COST_H