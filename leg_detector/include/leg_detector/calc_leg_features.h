#ifndef _CALC_LEG_FEATURES_H
#define _CALC_LEG_FEATURES_H

#include "laser_processor.h"
#include "sensor_msgs/LaserScan.h"

// TODO: Add description here
std::vector<float> calcLegFeatures(laser_processor::SampleSet*, const sensor_msgs::LaserScan&);

#endif
