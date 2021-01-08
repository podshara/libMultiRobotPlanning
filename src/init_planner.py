#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion
import time
import math
from mushr_coordination.msg import GoalPoseArray
from std_msgs.msg import String

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


if __name__ == "__main__":
    rospy.init_node("init_planner")
    pub = rospy.Publisher('chatter', String, queue_size=10)    
    hello_str = "hello world"
    rospy.sleep(1)
    pub.publish(hello_str)
    rospy.loginfo(hello_str)
    now = rospy.Time.now()
    cur_pose = PoseStamped()
    cur_pose.header.frame_id = "/map"
    cur_pose.header.stamp = now

    count = 4
    pubs = []
    # this is basically initializing all the subscribers for counting the number of cars and publishers for initiailizing pose and goal points.
    for i in range(count):
        publisher = rospy.Publisher("/car" + str(i+1) + "/car_pose", PoseStamped, queue_size=1)
        pubs.append(publisher)
    goal_pub = rospy.Publisher("/mushr_coordination/goals", GoalPoseArray, queue_size=1)
    rospy.sleep(1)

    car_pose = [[0, 10], [30, 3], [14, 11], [18, 10]]
    goalp = [[[6, 28], [32, 1]], [[25, 23], [7, 11]], [[21, 10], [10, 21]], [[5, 6], [31, 15]]]

    for i in range(count):
        cur_pose.pose.position.x = car_pose[i][0]
        cur_pose.pose.position.y = car_pose[i][1]
        cur_pose.pose.position.z = 0.0
        #cur_pose.pose.orientation = angle_to_quaternion(rot)
        print(cur_pose)
        pubs[i].publish(cur_pose)
    #rospy.spin()

    now = rospy.Time.now()
    goal_pose = GoalPoseArray()
    goal_pose.header.frame_id = "/map"
    goal_pose.header.stamp = now
    R = 2*math.sqrt(2)
    for i in range(count):
        goal_pose.goals.append(PoseArray())
        for j in range(2):
            goal = Pose()
            goal.position.x = goalp[i][j][0]
            goal.position.y = goalp[i][j][1]
            goal.position.z = 0.0
            goal_pose.goals[i].poses.append(goal)
    goal_pub.publish(goal_pose)
    #rospy.spin()
