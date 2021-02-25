#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from ackermann_msgs.msg import AckermannDriveStamped
# from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Quaternion, Pose, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class FollowWaypoints:
    def __init__(self):
        # Initalize anonoymous node
        rospy.init_node("waypoint_follower", anonymous=True)

        self.ackermann_cmd_topic = rospy.get_param("~ackermann_cmd_topic", "/ackermann_cmd")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.new_wp_topic = rospy.get_param("~new_wp_topic", "/waypoints/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.update_rate = rospy.get_param("~update_rate", 10)
        self.rate = rospy.Rate(self.update_rate)

        self.curr_x = rospy.get_param("~starting_x", 0)
        self.curr_y = rospy.get_param("~starting_y", 0)
        self.goal_x = rospy.get_param("~starting_x", 0)
        self.goal_y = rospy.get_param("~starting_y", 0)
        self.yaw = 0

        rospy.Subscriber(self.new_wp_topic, Pose, self.new_wp_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self._publish_cmd_vel()

        rospy.logwarn("Created waypoint follower; ready to receive waypoints")

    def odom_callback(self, data):
        self.curr_x = data.pose.pose.position.x
        self.curr_y = data.pose.pose.position.y

    def new_wp_callback(self, data):
        """Take in new waypoint to navigate to and determine ackermann command
        to send.
        """
        self.goal_x = data.position.x
        self.goal_y = data.position.y

        orientation = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        )

        _, _, self.yaw = euler_from_quaternion(orientation)

        rospy.logwarn(f"Received new waypoint goal: ({self.goal_x}, {self.goal_y}) -- heading: {self.yaw}")

    def _publish_cmd_vel(self):
        """Continuously publish ackermann command to vehicle"""
        new_cmd_vel = Twist()
        while not rospy.is_shutdown():
            inc_x  = round(self.goal_x - self.curr_x, 2)
            inc_y  = round(self.goal_y - self.curr_y, 2)
            rospy.logwarn(f"x: {inc_x} -- y: {inc_y}")

            distance = (inc_x**2 + inc_y**2) ** (1/2)
            angle_to_goal = math.atan2(inc_y, inc_x)

            angle_diff = angle_to_goal - self.yaw
            angle_sign = 1 if angle_diff > 0 else -1
            # extremly simple sterring controller
            if abs(angle_diff) > 0.1:
                new_cmd_vel.angular.z = 0.1 * angle_sign
            else:
                new_cmd_vel.angular.z = 0.0

            # extremely simple velocity controller
            sign = 1 if distance > 0 else -1
            if abs(distance) > 0.2:
                new_cmd_vel.linear.x = 0.5 * sign
            elif abs(distance) < 0.2:
                # new_cmd_vel.linear.x = 1 / distance
                new_cmd_vel.linear.x = 0
            else:
                new_cmd_vel.linear.x = 0

            self.pub_cmd_vel.publish(new_cmd_vel)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        FollowWaypoints()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
