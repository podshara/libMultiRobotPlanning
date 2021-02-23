#ifndef MUSHR_COORDINATION_H
#define MUSHR_COORDINATION_H

#include <iostream>
#include <algorithm>
#include <string>
#include <utility>
#include <yaml-cpp/yaml.h>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

//#include "libMultiRobotPlanning/ecbs_ta.hpp"
#include "libMultiRobotPlanning/next_best_assignment.hpp"
#include "mushr_coordination/GoalPoseArray.h"
#include "mushr_coordination/ecbs_mushr.hpp"


#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"


// TODO: need namespace for the header
using libMultiRobotPlanning::ECBSTA;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::NextBestAssignment;

namespace node_mushr_coor{
class MushrCoordination {
  public:
    MushrCoordination(ros::NodeHandle &nh)
      : m_maxTaskAssignments(1e9),
        m_planning(false),
        m_ini_obs(false),
        m_ini_goal(false),
        m_sim(true) {
      nh.getParam("/mushr_coordination/w", m_w);
      nh.getParam("/mushr_coordination/num_waypoint", m_num_waypoint);
      nh.getParam("/mushr_coordination/num_agent", m_num_agent);
      m_car_pose = std::vector<std::pair<double, double>>(m_num_agent);
      for (size_t i = 0; i < m_num_agent; ++i) {
        m_sub_car_pose.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
              "/car" + std::to_string(i + 1) + "/" + (m_sim ? "init_pose" : "mocap_pose"),
              10,
              boost::bind(&MushrCoordination::CarPoseCallback, this, _1, i)
            ));
        m_pub_plan.push_back(nh.advertise<geometry_msgs::PoseArray>(
              "/car" + std::to_string(i + 1) + "/waypoints", 
              10
            ));
        m_pub_pick.push_back(nh.advertise<visualization_msgs::Marker>(
              "/car" + std::to_string(i + 1) + "/pick", 
              10
            ));
        m_pub_drop.push_back(nh.advertise<visualization_msgs::Marker>(
              "/car" + std::to_string(i + 1) + "/drop", 
              10
            ));
      }
      m_sub_obs_pose = nh.subscribe("/mushr_coordination/obstacles", 
              10, 
              &MushrCoordination::ObsPoseCallback, 
              this);
      m_sub_goal = nh.subscribe("/mushr_coordination/goals", 
              10, 
              &MushrCoordination::GoalCallback, 
              this);
      ros::Duration(1).sleep();
    }

    void CarPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, size_t i) {
      std::cout << "get car " << i << std::endl;
      m_assigned.insert(i);
      m_car_pose[i] = std::make_pair(msg->pose.position.x, msg->pose.position.y);
      if (isReady()) {
          solve();
      }
    }

    void ObsPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
      std::cout << "get obs " << std::endl;
      m_ini_obs = true;  
      for (auto &pose : msg->poses) {
        m_obs_pose.emplace_back(pose.position.x, pose.position.y);
      }
      if (isReady()) {
        solve();
      }
    }

    void GoalCallback(const mushr_coordination::GoalPoseArray::ConstPtr& msg) {
      std::cout << "get goal" << std::endl;
      m_ini_goal = true;
      m_scale = msg->scale;
      m_minx = msg->minx;
      m_miny = msg->miny;
      m_maxx = msg->maxx;
      m_maxy = msg->maxy;
      for (auto &goal : msg->goals) {
        m_goal_pose.emplace_back();
        for (auto &pose : goal.poses) {
          m_goal_pose.back().emplace_back(pose.position.x, pose.position.y);
        }
      }
      if (isReady()) {
          solve();
      }
    }

  private:
    void solve() {
      m_planning = true;
      std::cout << "start planning" << std::endl;
      std::unordered_set<Location> obstacles;
      std::vector<State> startStates;
      std::vector<Waypoints> goals;
      // init goals
      for (auto& goal: m_goal_pose) {
        std::vector<Location> ls;
        for(auto& waypoint: goal) {
          ls.emplace_back(scalex(waypoint.first), scaley(waypoint.second));
        }
        goals.emplace_back(ls);
      }
      for (auto & g: goals) {
        std::cout << g << std::endl;
      }
      // init obstacles
      for(auto& pos: m_obs_pose) {
        obstacles.insert(Location(scalex(pos.first), scaley(pos.second)));
      }
      // init start states
      for(auto& pos: m_car_pose) {
        startStates.emplace_back(0, scalex(pos.first), scaley(pos.second), 0); 
      }

      int mkid = 0; //visualize
      double color[][4] = {{1, 0, 0, 1}, {0, 1, 0, 1}, {0, 0, 1, 1}, {1, 1, 0, 1}, {1, 0, 1, 1}, {0, 1, 1, 1}};

      int dimx = scalex(m_maxx) + 1;
      int dimy = scaley(m_maxy) + 1;
      std::cout << dimx << " " << dimy << " " << m_num_agent << " " << m_num_waypoint << std::endl;
      Environment mapf(dimx, dimy, m_num_waypoint, obstacles, startStates, goals,
                    m_maxTaskAssignments);
      std::cout << "done init environment" << std::endl;
      ECBSTA<State, Action, int, Conflict, Constraints, Waypoints,
            Environment>
          cbs(mapf, m_w);
      
      std::vector<PlanResult<State, Action, int> > solution;
      bool success = cbs.search(startStates, solution);
      std::cout << "planner " << (success ? "success" : "failed") << std::endl;

      if (success) {
        for (size_t a = 0; a < solution.size(); ++a) {
          geometry_msgs::PoseArray plan;
          plan.header.stamp = ros::Time::now();
          plan.header.frame_id = "map";
          int time = 1;
          for (size_t i = 0; i < solution[a].states.size(); i++) {
            geometry_msgs::Pose p;
            // visualize
            switch (solution[a].actions[i].first) {
              case Action::Up: 
                p.orientation.x = 0;
                p.orientation.y = 0;
                p.orientation.z = 0.707;
                p.orientation.w = 0.707;
                break;
              case Action::Down:
                p.orientation.x = 0;
                p.orientation.y = 0;
                p.orientation.z = 0.707;
                p.orientation.w = -0.707;
                break;
              case Action::Left: 
                p.orientation.x = 0;
                p.orientation.y = 0;
                p.orientation.z = 1;
                p.orientation.w = 0;
                break;
              case Action::Right:
                p.orientation.x = 0;
                p.orientation.y = 0;
                p.orientation.z = 0;
                p.orientation.w = 1; 
                break;
            //
              case Action::Wait:
                if (i < solution[a].states.size() - 1) {
                  time++;
                  continue;
                }
                break;
            }
                        
            double x = r_scalex(solution[a].states[i].first.x);
            double y = r_scaley(solution[a].states[i].first.y);
            p.position.x = x;
            p.position.y = y;
            p.position.z = 0; //time * 0.001;

            plan.poses.push_back(p);
            time = 1;
          }

          // visualize
          visualization_msgs::Marker pick;
          visualization_msgs::Marker drop;
          double marker_size = 0.25; 
          for (size_t i = 0; i < goals.size(); i++) {
            if (goals[i].points.back().x == solution[a].states.back().first.x &&
                goals[i].points.back().y == solution[a].states.back().first.y) {
              pick.pose.position.x = m_goal_pose[i][0].first;
              pick.pose.position.y = m_goal_pose[i][0].second;
              pick.pose.position.z = 0;
              drop.pose.position.x = m_goal_pose[i][1].first;
              drop.pose.position.y = m_goal_pose[i][1].second;
              drop.pose.position.z = 0;

              pick.color.r = color[a][0];
              pick.color.g = color[a][1];
              pick.color.b = color[a][2];
              pick.color.a = color[a][3];
              drop.color.r = color[a][0];
              drop.color.g = color[a][1];
              drop.color.b = color[a][2];
              drop.color.a = color[a][3];

              pick.scale.x = marker_size;
              pick.scale.y = marker_size;
              pick.scale.z = marker_size;
              drop.scale.x = marker_size;
              drop.scale.y = marker_size;
              drop.scale.z = marker_size;

              pick.header.frame_id = "map";
              pick.header.stamp = ros::Time();
              pick.id = mkid++;
              pick.type = visualization_msgs::Marker::SPHERE;
              pick.action = visualization_msgs::Marker::ADD;
              drop.header.frame_id = "map";
              drop.header.stamp = ros::Time();
              drop.id = mkid++;
              drop.type = visualization_msgs::Marker::SPHERE;
              drop.action = visualization_msgs::Marker::ADD;

              m_pub_pick[a].publish(pick);
              m_pub_drop[a].publish(drop);
              break;
            }
          }
          // 

          m_pub_plan[a].publish(plan);
          std::cout << "publish plan for car " << a+1 << std::endl;
        }
      }
      m_goal_pose.clear();
      m_obs_pose.clear();
      m_car_pose = std::vector<std::pair<double, double>>(m_num_agent);
      m_ini_obs = false;
      m_ini_goal = false;
      m_assigned.clear();
      m_planning = false;
    }

    bool isReady() {
      return m_assigned.size() == m_num_agent && m_ini_obs && m_ini_goal && !m_planning;
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
    std::vector<ros::Publisher> m_pub_pick;
    std::vector<ros::Publisher> m_pub_drop;
    std::vector<std::pair<double, double>> m_car_pose;
    std::vector<std::pair<double, double>> m_obs_pose;
    std::vector<std::vector<std::pair<double, double>>> m_goal_pose;
    std::set<size_t> m_assigned;
    bool m_planning;
    bool m_ini_obs;
    bool m_ini_goal;
    int m_maxTaskAssignments;
    int m_num_agent;
    int m_num_waypoint;
    bool m_sim;
    double m_w;
    double m_scale;
    double m_minx, m_miny, m_maxx, m_maxy;
};
}


#endif // MUSHR_COORDINATION_H