#pragma once

#include "stdafx.h"
#include "helpers.h"

using namespace std;

class PathPlanner
{
  public:

    PathPlanner() : m_rep_ctr(0) {}

    Trajectory getPath(VehicleData &veh_data,
                       const vector<double> &maps_x,
                       const vector<double> &maps_y,
                       const vector<double> &maps_s,
                       const vector<double> &maps_dx,
                       const vector<double> &maps_dy);

  private:
    Trajectory calculateTrajectory(Trajectory rough_traj,
                                   double ref_x,
                                   double ref_y,
                                   double ref_yaw,
                                   double target_vel,
                                   int num_steps,
                                   const VehicleData &veh_data,
                                   const MapData &map_data);

    std::vector<double> getLaneSpeeds(const std::vector<FusionData> &veh_in_lanes, const double &s);

    std::vector<FusionObjData> getVehiclesFront(const std::vector<FusionData> &veh_in_lanes, const double &s);

    Predictions getTrajectoryPredictions(const FusionData &vehicles, int num_steps, MapData map_data);

    bool checkIfLaneChangePossible(const VehicleData &veh_data,
                                   const FusionData &vehicles,
                                   int target_lane,
                                   double target_vel,
                                   const MapData &map_data);

    Trajectory getRoughTrajectoryStart(const VehicleData &veh_data);

  private:
    enum eState
    {
        FOLLOWLANE,
        PREPARELANECHANGE,
        CHANGELANE
    };

    eState m_state;
    int m_target_lane;
    int m_tmp_target_lane;
    int m_rep_ctr;
};