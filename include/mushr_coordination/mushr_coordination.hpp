#ifndef MUSHR_COORDINATION_H
#define MUSHR_COORDINATION_H

#include <iostream>
#include <algorithm>
#include <string>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include "libMultiRobotPlanning/ecbs_ta.hpp"
#include "libMultiRobotPlanning/next_best_assignment.hpp"
#include "mushr_coordination/GoalPoseArray.h"
#include "mushr_coordination/ecbs_mushr.hpp"


#include "ros/ros.h"
#include "ros/time.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


// TODO: need namespace for the header
using libMultiRobotPlanning::ECBSTA;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::NextBestAssignment;

namespace node_mushr_coor{
class MushrCoordination {
  public:
    MushrCoordination(ros::NodeHandle &nh)  {
      // nh.getParam("minx", m_minx);
      // nh.getParam("miny", m_miny);
      // nh.getParam("maxx", m_maxx);
      // nh.getParam("maxy", m_maxy);
      // if(!nh.getParam("w", m_w)) {m_w = 1.0;} // suboptimality
      m_w = 1.0;
      m_maxTaskAssignments = 1e9;
      m_scale = 1.0;
      m_num_agent = 8;
      m_sim = true;
      // std::string param;
      //if(!nh.getParam("maxTaskAssignments", m_maxTaskAssignments)) {m_maxTaskAssignments = 1e9;}
      // if(!nh.getParam("scale", m_scale)) {m_scale = 1.0;}
      // if(!nh.getParam("numAgent", m_num_agent)) {m_num_agent = 8;}
      // if(!nh.getParam("sim", param)) {m_sim = param.compare("true");}
      m_startStates = std::vector<State>();
      for (size_t i = 0; i < m_num_agent; ++i) {
        m_startStates.emplace_back(-1, -1, -1, -1);
      }
      for (size_t i = 0; i < m_num_agent; ++i) {
        m_sub_car_pose.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
              "/car" + std::to_string(i + 1) + "/" + (m_sim ? "car_pose" : "mocap_pose"),
              10,
              boost::bind(&MushrCoordination::CarPoseCallback, this, _1, i)
            ));
        m_pub_plan.push_back(nh.advertise<geometry_msgs::PoseArray>(
              "/car" + std::to_string(i + 1) + "/waypoints", 
              10
            ));
      }
      // TODO: subscribe to obstracle topic : PoseArray
      m_sub_obs_pose = nh.subscribe("/mushr_coordination/obstacles", 
              10, 
              &MushrCoordination::ObsPoseCallback, 
              this);
      // TODO: subscribe to goal: array of PoseArray
      // m_sub_goal = nh.subscribe("/mushr_coordination/goals", 10, &MushrCoordination::GoalCallback, this);
    }

    void CarPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, size_t i) {
      m_assigned.insert(i);
      m_startStates[i] = State(0, scalex(msg->pose.position.x), scaley(msg->pose.position.y), 0);
      if (isReady()) {
          solve();
      }
    }

    void ObsPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
      m_ini_obs = true;
      for (auto &pose : msg->poses) {
        m_obstacles.insert(Location(scalex(pose.position.x), scaley(pose.position.y)));
      }
      if (isReady()) {
        solve();
      }
    }

    void GoalCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
      // m_ini_obs = true;
      // std::vector<Location> tmp;
      // for (auto &goal : msg->goals) {
      //   for (auto &pose : goal->poses) {
      //     tmp.insert(getLocation(pose));
      //   }
      //   m_goal.emplace_back(tmp);
      //   tmp.clear();
      // }
      // if (isReady()) {
      //     solve();
      // }
    }

  private:
    void solve() {
      int dimx = scalex(m_maxx);
      int dimy = scaley(m_maxy);
      Environment mapf(dimx, dimy, 1, m_obstacles, m_startStates, m_goals,
                    m_maxTaskAssignments);
      ECBSTA<State, Action, int, Conflict, Constraints, Waypoints,
            Environment>
          cbs(mapf, m_w);
      std::vector<PlanResult<State, Action, int> > solution;
      bool success = cbs.search(m_startStates, solution);

      if (success) {
        for (size_t a = 0; a < solution.size(); ++a) {
          geometry_msgs::PoseArray plan;
          plan.header.stamp = ros::Time::now();
          plan.header.frame_id = "map";
          for (const auto& state : solution[a].states) {
            geometry_msgs::Pose p;
            p.position.x = r_scalex(state.first.x);
            p.position.y = r_scaley(state.first.y);
            p.position.z = 0.0;
            plan.poses.push_back(p);
          }
          m_pub_plan[a].publish(plan);
        }
      }
    }

    bool isReady() {
      return m_assigned.size() == m_num_agent /*&& m_ini_obs*/ && !m_startStates.empty() && !m_goals.empty();
    }

    int scalex(double x) {
      return floor((x - m_minx) * m_scale);
    }

    int scaley(double y) {
      return floor((y - m_miny) * m_scale);
    }

    double r_scalex(int x) {
      return x / m_scale + m_minx;
    }

    double r_scaley(int y) {
      return y / m_scale + m_miny;
    }

    bool goal_compare(Waypoints & obj, int x, int y) {
      return obj.points.back().x == x && obj.points.back().y == y;
    }

    std::vector<ros::Subscriber> m_sub_car_pose;
    ros::Subscriber m_sub_obs_pose;
    ros::Subscriber m_sub_goal;
    std::vector<ros::Publisher> m_pub_plan;
    std::unordered_set<Location> m_obstacles;
    std::vector<State> m_startStates;
    std::vector<Waypoints> m_goals;
    // Environment m_map;
    // ECBSTA<State, Action, int, Conflict, Constraints, Waypoints, Environment> cbs;
    std::set<size_t> m_assigned;
    bool m_ini_obs;
    size_t m_maxTaskAssignments;
    size_t m_num_agent;
    bool m_sim;
    double m_w;
    double m_scale;
    double m_minx = -20, m_miny = -20, m_maxx = 20, m_maxy = 20;
};
}


#endif // MUSHR_COORDINATION_H