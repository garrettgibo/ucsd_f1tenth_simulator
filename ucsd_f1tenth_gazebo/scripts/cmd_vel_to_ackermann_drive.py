#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


class ConvertCmd:
    def __init__(self):
        # Initalize anonoymous node
        rospy.init_node("cmd_vel_to_ackermann_drive", anonymous=True)

        self.twist_cmd_topic = rospy.get_param("~twist_cmd_topic", "/cmd_vel")
        self.ackermann_cmd_topic = rospy.get_param("~ackermann_cmd_topic", "/ackermann_cmd")
        self.wheelbase = rospy.get_param("~wheelbase", 1.0)
        self.frame_id = rospy.get_param("~frame_id", "odom")

        self.message_type = rospy.get_param(
            "~message_type", "ackermann_drive"
        )  # ackermann_drive or ackermann_drive_stamped

        rospy.Subscriber(self.twist_cmd_topic, Twist, self.cmd_callback, queue_size=1)

        self.pub = rospy.Publisher(self.ackermann_cmd_topic, AckermannDrive, queue_size=1)

        rospy.logwarn(
            "Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f",
            "/cmd_vel",
            self.ackermann_cmd_topic,
            self.frame_id,
            self.wheelbase,
        )

    def cmd_callback(self, data):
        v = data.linear.x
        steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, self.wheelbase)

        msg = AckermannDrive()
        msg.steering_angle = steering
        msg.speed = v

        self.pub.publish(msg)


if __name__ == "__main__":
    try:
        ConvertCmd()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
