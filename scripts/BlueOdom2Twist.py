#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
import tf
import numpy as np

odom_pub = rospy.Publisher('/bluerov2/DVL_odom', Odometry, queue_size=10)


def get_homo_matrix_from_odom(Odom):
    translation = tf.transformations.translation_matrix(
        (Odom.pose.pose.position.x, Odom.pose.pose.position.y, Odom.pose.pose.position.z))
    orientation = tf.transformations.quaternion_matrix(
        (Odom.pose.pose.orientation.x, Odom.pose.pose.orientation.y, Odom.pose.pose.orientation.z,
         Odom.pose.pose.orientation.w))
    combined = np.matmul(translation, orientation)
    translation_from_m = tf.transformations.translation_from_matrix(combined)
    orientation_from_m = tf.transformations.quaternion_from_matrix(combined)
    # rospy.loginfo("odom translation: " + str(translation_from_m))
    # rospy.loginfo("odom quaternion: " + str(orientation_from_m))
    return combined


def inverse_homo_matrix(T):
    R_a_b = T[0:3, 0:3]
    t_a_ab = T[0:3, 3].reshape(3, 1)
    R_b_a = np.transpose(R_a_b)
    t_b_ba = -R_b_a.dot(t_a_ab)
    combined = np.block([[R_b_a, t_b_ba], [np.zeros((1, 3)), 1]])
    return combined


def twist_cb(data):
    T_b0_bj = get_homo_matrix_from_odom(data)
    T_bj_b0 = inverse_homo_matrix(T_b0_bj)
    v_b0 = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
    v_b0.resize((3, 1))
    R_bj_b0 = T_bj_b0[0:3, 0:3]
    v_bj = R_bj_b0.dot(v_b0) * 100
    odom = Odometry()
    odom.header.stamp = data.header.stamp
    odom.header.frame_id = data.header.frame_id
    odom.twist = data.twist
    odom.twist.twist.linear.x = v_bj[0]
    odom.twist.twist.linear.y = v_bj[1]
    odom.twist.twist.linear.z = v_bj[2]
    odom_pub.publish(odom)


if __name__ == '__main__':
    rospy.init_node('twist2odom', anonymous=True)
    rospy.Subscriber("/bluerov2/odometry", Odometry, twist_cb)
    rospy.spin()
