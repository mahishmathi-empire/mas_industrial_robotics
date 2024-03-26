#ifndef TWIST_SYNCHRONIZER_UTILS_HPP
#define TWIST_SYNCHRONIZER_UTILS_HPP
#include <algorithm>
#include <cmath>
#include <cassert>
#include <vector>

double calculate_max_time(const std::vector<double>& error, const std::vector<double>& velocity, bool angularSynchronization, double zero);

std::vector<double> calculate_sync_velocity(const std::vector<double>& error, const std::vector<double>& velocity, double maxTime, bool angularSynchronization);

double cal_dist(double distance, double speed);

int sign(double val) ;

#endif // TWIST_SYNCHRONIZER_HPP
