#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped

odom_pub = rospy.Publisher('/rexrov2/DVL', Odometry, queue_size=10)


def twist_cb(data):
    odom = Odometry()
    odom.header.stamp = data.header.stamp
    odom.header.frame_id = data.header.frame_id
    odom.twist = data.twist
    odom_pub.publish(odom)


if __name__ == '__main__':
    rospy.init_node('twist2odom', anonymous=True)
    rospy.Subscriber("/rexrov2/dvl_twist", TwistWithCovarianceStamped, twist_cb)
    rospy.spin()
