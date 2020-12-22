cmake_minimum_required(VERSION 2.8.3)
project(mushr_coordination)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

add_message_files(
  FILES
  GoalPoseArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    tf2
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_LIBRARIES}
  libMultiRobotPlanning/include
)



# set(ALL_SOURCE_FILES
#   libMultiRobotPlanning/include/libMultiRobotPlanning/a_star.hpp
#   libMultiRobotPlanning/include/libMultiRobotPlanning/a_star_epsilon.hpp
#   libMultiRobotPlanning/include/libMultiRobotPlanning/assignment.hpp
#   libMultiRobotPlanning/include/libMultiRobotPlanning/cbs.hpp
#   libMultiRobotPlanning/include/libMultiRobotPlanning/cbs_ta.hpp
#   libMultiRobotPlanning/include/libMultiRobotPlanning/ecbs.hpp
#   libMultiRobotPlanning/include/libMultiRobotPlanning/ecbs_ta.hpp
#   libMultiRobotPlanning/include/libMultiRobotPlanning/neighbor.hpp
#   libMultiRobotPlanning/include/libMultiRobotPlanning/next_best_assignment.hpp
#   libMultiRobotPlanning/include/libMultiRobotPlanning/planresult.hpp
#   libMultiRobotPlanning/include/libMultiRobotPlanning/sipp.hpp
#   libMultiRobotPlanning/example/timer.hpp
# )

# #add_executable(libMultiRobotPlanning ${ALL_SOURCE_FILES})
#target_link_libraries(libMultiRobotPlanning ${Boost_LIBRARIES})

# ## a_star
# add_executable(a_star
#   libMultiRobotPlanning/example/a_star.cpp
# )
# target_link_libraries(a_star
#   ${Boost_LIBRARIES}
# )

# ## a_star_epsilon
# add_executable(a_star_epsilon
#   libMultiRobotPlanning/example/a_star_epsilon.cpp
# )
# target_link_libraries(a_star_epsilon
#   ${Boost_LIBRARIES}
# )

# ## assignment
# add_executable(assignment
#   libMultiRobotPlanning/example/assignment.cpp
# )
# target_link_libraries(assignment
#   ${Boost_LIBRARIES}
# )

# ## cbs
# add_executable(cbs
#   libMultiRobotPlanning/example/cbs.cpp
# )
# target_link_libraries(cbs
#   ${Boost_LIBRARIES}
#   yaml-cpp
# )

# ## cbs_ta
# add_executable(cbs_ta
#   libMultiRobotPlanning/example/cbs_ta.cpp
# )
# target_link_libraries(cbs_ta
#   ${Boost_LIBRARIES}
#   yaml-cpp
# )

# ## ecbs
# add_executable(ecbs
#   libMultiRobotPlanning/example/ecbs.cpp
# )
# target_link_libraries(ecbs
#   ${Boost_LIBRARIES}
#   yaml-cpp
# )

# ## ecbs_ta
# add_executable(ecbs_ta
#   libMultiRobotPlanning/example/ecbs_ta.cpp
# )
# target_link_libraries(ecbs_ta
#   ${Boost_LIBRARIES}
#   yaml-cpp
# )

# ## ecbs_ta
# add_executable(ecbs_mushr
#   libMultiRobotPlanning/example/ecbs_mushr.cpp
# )
# target_link_libraries(ecbs_mushr
#   ${Boost_LIBRARIES}
#   yaml-cpp
# )

# ## next_best_assignment
# add_executable(next_best_assignment
#   libMultiRobotPlanning/example/next_best_assignment.cpp
# )
# target_link_libraries(next_best_assignment
#   ${Boost_LIBRARIES}
#   yaml-cpp
# )

# ## sipp
# add_executable(sipp
#   libMultiRobotPlanning/example/sipp.cpp
# )
# target_link_libraries(sipp
#   ${Boost_LIBRARIES}
#   yaml-cpp
# )

# ## mapf_prioritized_sipp
# add_executable(mapf_prioritized_sipp
#   libMultiRobotPlanning/example/mapf_prioritized_sipp.cpp
# )
# target_link_libraries(mapf_prioritized_sipp
#   ${Boost_LIBRARIES}
#   yaml-cpp
# )

add_executable(mushr_coordination src/nodes/mushr_coor_node.cpp src/mushr_coordination.cpp)
target_link_libraries(mushr_coordination ${catkin_LIBRARIES})
#add_dependencies(mushr_coordination libMultiRobotPlanning)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY libMultiRobotPlanning/include/libMultiRobotPlanning/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

#add_library(libMultiRobotPlanning)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#