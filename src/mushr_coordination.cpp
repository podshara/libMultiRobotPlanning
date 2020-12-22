// #include "mushr_coordination/mushr_coordination.h"

// //namespace node_mushr_coor{
// MushrCoordination::MushrCoordination(ros::NodeHandle &nh) {
//   nh.getParam("minx", m_minx);
//   nh.getParam("miny", m_miny);
//   nh.getParam("maxx", m_maxx);
//   nh.getParam("maxy", m_maxy);
//   if(!nh.getParam("w", m_w)) {m_w = 1.0;} // suboptimality
//   if(!nh.getParam("maxTaskAssignments", m_maxTaskAssignments)) {m_maxTaskAssignments = 1e9;}
//   if(!nh.getParam("scale", m_scale)) {m_scale = 1.0;}
//   if(!nh.getParam("numAgent", m_num_agent)) {m_num_agent = 8;}
//   if(!nh.getParam("sim", m_sim)) {sum = true;}
//   m_startStates = std::vector<auto>(m_num_agent);
//   for (size_t i = 0; i < m_num_agent; ++i) {
//     m_sub_car_pose.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
//           "/car{}/{}".format(i + 1, m_sim ? "car_pose" : "mocap_pose"),
//           10,
//           boost::bind(&MushrCoordination::CarPoseCallback, this, _1, i);
//         ));
//   }
//   // TODO: subscribe to obstracle topic : PoseArray
//   m_sub_obs_pose = nh.subscribe<geometry_msgs::PoseArray>("/mushr_coordination/obstacles", 10, &MushrCoordination::ObsPoseCallback);
//   // TODO: subscribe to goal: array of PoseArray
//   m_sub_goal = nh.subscribe<mushr_coordination::GoalPoseArray>("/mushr_coordination/goals", 10, &MushrCoordination::GoalCallback);
//   // TODO: add planner publisher
//   m_pub_plan = nh.advertise<geometry_msgs::PoseArray>("/mushr_coordination/plan", 10);
// }

// void MushrCoordination::CarPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, size_t i) {
//   m_assigned.insert(i);
//   m_startStates[i] = getLocation(msg);
//   if (Private::isReady()) {
//       Private::solve();
//   }
// }

// void MushrCoordination::ObsPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
//   m_ini_obs = true;
//   for (auto &pose : msg->poses) {
//     m_obstacles.insert(getLocation(pose));
//   }
//   if (Private::isReady()) {
//     Private::solve();
//   }
// }

// void MushrCoordination::GoalCallback(const mushr_coordination::GoalPoseArray::ConstPtr& msg) {
//   m_ini_obs = true;
//   std::vector<Location> tmp;
//   for (auto &goal : msg->goals) {
//     for (auto &pose : goal->poses) {
//       tmp.insert(getLocation(pose));
//     }
//     m_goal.emplace_back(tmp);
//     tmp.clear();
//   }
//   if (Private::isReady()) {
//       Private::solve();
//   }
// }

// struct MushrCoordination::Private {
//   void solve() {
//     m_map = Environment(scalex(m_maxx), scaley(m_maxy), 1 /*num waypoint*/, 
//       m_obstacles, m_startStates, m_goals, m_maxTaskAssignments);
//     cbs = ECBSTA<auto>(m_map, m_w);
//     std::vector<PlanResult<State, Action, int> > sub_solution;
//     bool success = cbs.search(startStates, sub_solution);

//     // TODO: publish plan
//     /*
//     mushr_coordination::PoseArray pub;
//     pub.header.stamp = ros::Time::now();
//     pub.header.frame_id = "map";
//     pub.success = success;
//     if (success) {
//       for (size_t a = 0; a < solution.size(); ++a) {
//         for (const auto& state : solution[a].states) {
//           geometry_msgs::Pose p;
//           p.position.x = r_scalex(state.first.x);
//           p.position.y = r_scaley(state.first.y);
//           p.position.z = 0.0;
//           //pub.plans.push_back(p);
//         }
//       }
//     }
//     m_pub_plan.publish(pub);
//     */
//   }

//   Location getLocation(const geometry_msgs::PoseStamped::ConstPtr& pose) {
//     return Location(scalex(pose.pose.position.x), scaley(pose.pose.position.y));
//   }

//   bool isReady() {
//     return m_assigned.size() == m_num_agent && m_ini_obs && !m_startStates.empty() && !m_goal.empty();
//   }

//   int scalex(double x) {
//     return floor((x - m_minx) * m_scale);
//   }

//   int scaley(double y) {
//     return floor((y - m_miny) * m_scale);
//   }

//   double r_scalex(int x) {
//     return x / m_scale + m_minx;
//   }

//   double r_scaley(int y) {
//     return y / m_scale + m_miny;
//   }

//   bool goal_compare(Waypoints & obj, int x, int y) {
//     return obj.points.back().x == x && obj.points.back().y == y;
//   }
// }
// //}
