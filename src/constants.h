#pragma once

// Maximum acceleration in m/s in a 0.02s time step
constexpr double a_max = 0.1;

// Number of lanes
constexpr int num_lanes = 3;

// Maximum velocityin mph
constexpr double v_max = 49.5;

// Width of the lanes
constexpr double lane_width = 4.0;

// Minimum distance between the end of the ego vehicle's planned trajectory and the vehicle in front
constexpr double dist_margin = 15.0;

// Half of the width of the ACC ROI
constexpr double ACC_ROI_LAT = 2.0;

// Num of cycles after the road is considered straight
constexpr int ROAD_CURV_CTR_MAX = 150;

// Minimum time the FOLLOWLANE state must stay active
constexpr int FOLLOW_LANE_MIN_TIME = 50;

// Minimum time the CHANGELANE state must stay active
constexpr int CHANGE_LANE_MIN_TIME = 50;

// Maximum time CHANGELANE state can stay active. State machine is forced to FOLLOWLANE state after it run out
constexpr int CHANGE_LANE_MAX_TIME = 500;

// Threshold for the road curvature to be considered curvy.
constexpr double ROAD_MAX_CURV_STRAIGHT = 3.0;