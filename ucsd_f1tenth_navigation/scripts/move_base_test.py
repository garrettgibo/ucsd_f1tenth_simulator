#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class MoveBaseClient:
    def __init__(self, map_frame="/map"):
        rospy.init_node('movebase_client_py')
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

        self.goalId = 0
        self.goalMsg = PoseStamped()

        self.goalMsg.header.frame_id = map_frame
        # self.goalMsg.pose.orientation.z = 0.0
        self.goalMsg.pose.orientation.w = 1.0

        self.goalMsg.header.stamp = rospy.Time.now()
        self.goalMsg.pose.position.x = 1
        self.goalMsg.pose.position.y = 1

        self.pub.publish(self.goalMsg)
        rospy.loginfo(f"Publishing Goal: {self.goalMsg}")

if __name__ == '__main__':
    try:
        MoveBaseClient()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
