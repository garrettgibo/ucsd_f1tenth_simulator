#!/usr/bin/env python

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class OdomTfPublisher:
    def __init__(self):
        # Initalize anonoymous node
        rospy.init_node("cmd_vel_to_ackermann_drive", anonymous=True)
        self.odom_broadcaster = tf.TransformBroadcaster()

        odom_topic = rospy.get_param("~odom_topic", "/odom")

        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)

        rospy.logwarn("Odometry Transform Publisher Created")

    def odom_callback(self, data):
        # rospy.logwarn(data)
        odom_pos = (data.pose.pose.position.x, data.pose.pose.position.y, 0)
        odom_quat = (data.pose.pose.orientation.x,
                     data.pose.pose.orientation.y,
                     data.pose.pose.orientation.z,
                     data.pose.pose.orientation.w)
        self.odom_broadcaster.sendTransform(
            odom_pos,
            odom_quat,
            rospy.Time.now(),
            "base_link",
            "odom"
        )


if __name__ == "__main__":
    try:
        OdomTfPublisher()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
