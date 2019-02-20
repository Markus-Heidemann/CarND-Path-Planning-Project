#pragma once

// Maximum acceleration in m/s in a 0.02s time step
constexpr double a_max = 0.10;

// Number of lanes
constexpr int num_lanes = 3;

// Maximum velocityin mph
constexpr double v_max = 49.5;

// Width of the lanes
constexpr double lane_width = 4.0;

// Minimum distance between the end of the ego vehicle's planned trajectory and the vehicle in front
constexpr double dist_margin = 15.0;