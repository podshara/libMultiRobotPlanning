#ifndef MUSHR_COORDINATION_H
#define MUSHR_COORDINATION_H

#include <iostream>
#include <algorithm>
#include "ros/ros.h"
#include "ros/time.h"
#include "ecbs_mushr.h"
#include "mushr_coordination/GoalPoseArray.h"
#include "mushr_coordination/PlanPoseArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// TODO: need namespace for the header

class MushrCoordination {
  public:
    explicit MushrCoordination(ros::NodeHandle &nh) {}

    void CarPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, size_t i);

    void ObsPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

    void GoalCallback(const mushr_coordination::GoalPoseArray::ConstPtr& msg);

  private:
    struct Private;
    std::vector<ros::Subscriber> m_sub_car_pose;
    ros::Subscriber m_sub_obs_pose;
    ros::Subscriber m_sub_goal;
    ros::Publisher m_pub_plan;
    std::vector<Location> m_obstacles;
    std::vector<Location> m_startStates;
    std::vector<Waypoint> m_goal;
    Environment m_map;
    ECBSTA<State, Action, int, Conflict, Constraints, Waypoints, Environment> cbs;
    std::set<size_t> m_assigned;
    bool m_ini_obs;
    size_t m_maxTaskAssignments;
    size_t m_num_agent;
    bool m_sim;
    double m_w;
    double m_scale;
    double m_minx, m_miny, m_maxx, m_maxy;
}


#endif // MUSHR_COORDINATION_H