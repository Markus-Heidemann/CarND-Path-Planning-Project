#pragma once

#include "stdafx.h"

using namespace std;

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

struct FusionObjData
{
    //[ id, x, y, vx, vy, s, d]
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;

    // void from_json(const nlohmann::json& j, FusionData& fus) {
    //     fus.id = j.at("test").get<A>();
    // }
};

typedef std::vector<FusionObjData> FusionData;

inline void from_json(const nlohmann::json &j, FusionData &fus_data)
{
    int fus_data_size = j.size();
    for (unsigned int i = 0; i < fus_data_size; ++i)
    {
        FusionObjData fus_obj;
        fus_obj.id = j[i][0];
        fus_obj.x = j[i][1];
        fus_obj.y = j[i][2];
        fus_obj.vx = j[i][3];
        fus_obj.vy = j[i][4];
        fus_obj.s = j[i][5];
        fus_obj.d = j[i][6];
        fus_data.push_back(fus_obj);
    }
}

struct VehicleData
{
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

    // Previous path data given to the Planner
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;
    // Previous path's end s and d values
    double end_path_s;
    double end_path_d;

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    FusionData sensor_fusion;
};

inline void from_json(const nlohmann::json &j, VehicleData &veh_data)
{
    // Main car's localization Data
    veh_data.car_x = j["x"];
    veh_data.car_y = j["y"];
    veh_data.car_s = j["s"];
    veh_data.car_d = j["d"];
    veh_data.car_yaw = j["yaw"];
    veh_data.car_speed = j["speed"];

    // Previous path data given to the Planner
    int prev_path_size = j["previous_path_x"].size();
    for (unsigned int i = 0; i < prev_path_size; ++i)
    {
        veh_data.previous_path_x.push_back(j["previous_path_x"][i]);
        veh_data.previous_path_y.push_back(j["previous_path_y"][i]);
    }
    // Previous path's end s and d values
    veh_data.end_path_s = j["end_path_s"];
    veh_data.end_path_d = j["end_path_d"];

    veh_data.sensor_fusion = j["sensor_fusion"].get<FusionData>();
}

inline double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

inline int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = min(2 * pi() - angle, angle);

  if (angle > pi() / 2)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
inline vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0)
  {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
inline vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}