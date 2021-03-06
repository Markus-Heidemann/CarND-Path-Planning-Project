#pragma once

#include "stdafx.h"
#include "helpers.h"

using namespace std;

class PathPlanner
{
  public:

    PathPlanner() : m_rep_ctr(0),
                    m_wp_offset(30.0),
                    m_road_curv_ctr(0),
                    m_follow_lane_ctr(0),
                    m_change_lane_ctr(0),
                    m_road_straight_b(false),
                    m_state(eState::FOLLOWLANE) {}

    /*!
     * main function of PathPlanner, that returns the trajectory of the ego vehicle based on
     * the data returned from the simulator and previous states
     */
    Trajectory getPath(VehicleData &veh_data,
                       const vector<double> &maps_x,
                       const vector<double> &maps_y,
                       const vector<double> &maps_s,
                       const vector<double> &maps_dx,
                       const vector<double> &maps_dy);

  private:
    /*!
     * calculates a trajectory of points, which are 0.02s apart from each other. The trajectory is
     * calulated based on a given "rough trajectory", e.g. waypoints, which can be 30m apart.
     * A spline is fitted through these points along which the fine trajectory is then generated.
     */
    Trajectory calculateTrajectory(Trajectory rough_traj,
                                   double ref_x,
                                   double ref_y,
                                   double ref_yaw,
                                   double target_vel,
                                   int num_steps,
                                   const VehicleData &veh_data,
                                   const MapData &map_data);

    /*!
     * Returns the speed in a lane based on the longitudinally nearest car in the lane in front of
     * the ego vehicle
     */
    std::vector<double> getLaneSpeeds(const std::vector<FusionData> &veh_in_lanes,
                                      const double &s,
                                      const double max_dist = INF);

    /*!
     * Gets the vehicles in each lane, that are longitudinally closest to and in front of the ego
     * vehicle
     */
    std::vector<FusionObjData> getVehiclesFront(const std::vector<FusionData> &veh_in_lanes,
                                                const double &s,
                                                const double max_dist = INF);

    /*!
     * Predicts a trajectory of each non-ego vehicle based on its current speed. The function
     * assumes, that the vehicle will follow its current lane with a constant velocity
     */
    Predictions getTrajectoryPredictions(const FusionData &vehicles, int num_steps, MapData map_data);

    /*!
     * Checks, if changing to a given lane is possible by predicting the trajectory of all non-ego
     * vehicles and calculating a possible ego trajectory. It then checks, if the ego-trajectory
     * comes to close to a trajectory of another vehicle.
     */
    bool checkIfLaneChangePossible(const VehicleData &veh_data,
                                   const FusionData &vehicles,
                                   int target_lane,
                                   double target_vel,
                                   const MapData &map_data);

    /*!
     * Calculates the first two waypoints of a rough trajectory. These waypoints are calculated in a
     * way, that make the connection between the previous trajectory and the new one smooth.
     */
    Trajectory getRoughTrajectoryStart(const VehicleData &veh_data);

    /*!
     * Calculates the target velocity for the ego vehicle based on the velocity of vehicles in front
     * in the same lane. The name ACC is inspired by 'Adaptive Cruise Control', since this function
     * basically implements the same functionality. The function uses Frenet coordinates.
     */
    double setACCVelByLane(vector<FusionData> fus_obj_by_lane, int curr_lane, double car_s,double end_path_s);

    /*!
     * Searches for vehicles in a region of interest (ROI) in front of the ego vehicle. If there are vehicles in the ROI
     * it choses the longitudinally cloest one and sets the target speed of the ego to the speed of this vehicle. This
     * function uses cartesion coordinates.
     */
    double setACCVel(const FusionData& fus_objs, const VehicleData& veh_data);

    /*!
     * Returns the indices of the lanes, that allow the highest speed.
     */
    vector<int> getFastestLane(std::vector<double> lane_speeds);

  private:
    enum eState
    {
        FOLLOWLANE,
        PREPARELANECHANGE,
        CHANGELANE
    };

    // Current state machine state
    eState m_state;

    // Index of lane, that the ego is trying to drive on
    int m_target_lane;

    // Intermediate target lane, if current lane and actual target lane are not right next to each
    // other
    int m_tmp_target_lane;

    // After a lane change, this counter has to run out, before a new lane change is allowed
    int m_rep_ctr;

    // Waypoint offset, which defines the distance between the support points for the trajectory
    // spline
    double m_wp_offset;

    // In order for the road ahead to be considered straight, the curvature has to be below a threshold for a certain
    // amount of time. This counter is used for that purpose.
    int m_road_curv_ctr;

    // This counter is used to determine how long the CHANGELANE state was active
    int m_change_lane_ctr;

    // This counter is used to determine how long the FOLLOWLANE state was active
    int m_follow_lane_ctr;

    // This bool is true, if the road ahead is straight.
    bool m_road_straight_b;
};