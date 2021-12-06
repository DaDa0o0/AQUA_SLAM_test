#! /usr/bin/env python
from urllib2 import urlopen
# import cv2
from pynput import keyboard
import numpy as np
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage, Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
from sensor_msgs.srv import SetCameraInfoResponse
import time

camera_info = CameraInfo()
img_pub_l = rospy.Publisher('/camera_black/img_l/compressed', CompressedImage, queue_size=10)
img_pub_r = rospy.Publisher('/camera_black/img_r/compressed', CompressedImage, queue_size=10)
b_set_loss = False


# info_pub = rospy.Publisher('/mi_camera/camera_info', CameraInfo, queue_size=10)


# def set_camera_para(req):
#     global camera_info
#     camera_info = req.camera_info
#     res = SetCameraInfoResponse()
#     res.status_message = 'success'
#     res.success = True
#     return res


def camera_cb_r(data):
    global img_pub_r
    global b_set_loss
    bridge_r = CvBridge()
    cv_img_r = bridge_r.compressed_imgmsg_to_cv2(data, 'bgr8')
    # data.header.stamp = rospy.Time.now()
    empty_img = np.random.randint(255, size=cv_img_r.shape, dtype=np.uint8)
    # image_msg_r = bridge_r.cv2_to_compressed_imgmsg(empty_img, 'jpeg compressed bgr8')
    image_msg = bridge_r.cv2_to_compressed_imgmsg(empty_img, 'jpeg')
    image_msg.header.stamp = data.header.stamp
    # img_pub_r.publish(image_msg_r)
    # rospy.Time.to_sec()
    # rospy.loginfo('image recieved, time: %f', data.header.stamp.to_sec())
    if b_set_loss:
        img_pub_r.publish(image_msg)
    else:
        img_pub_r.publish(data)

def camera_cb_l(data):
    global img_pub_l
    global b_set_loss
    bridge_r = CvBridge()
    cv_img_r = bridge_r.compressed_imgmsg_to_cv2(data, 'bgr8')
    # data.header.stamp = rospy.Time.now()
    empty_img = np.random.randint(255, size=cv_img_r.shape, dtype=np.uint8)
    # image_msg_r = bridge_r.cv2_to_compressed_imgmsg(empty_img, 'jpeg compressed bgr8')
    image_msg = bridge_r.cv2_to_compressed_imgmsg(empty_img, 'jpeg')
    image_msg.header.stamp = data.header.stamp
    # img_pub_r.publish(image_msg_r)
    # rospy.Time.to_sec()
    # rospy.loginfo('image recieved, time: %f', data.header.stamp.to_sec())
    if b_set_loss:
        img_pub_l.publish(image_msg)
    else:
        img_pub_l.publish(data)


def on_press(key):
    global b_set_loss
    if key == keyboard.Key.esc:
        rospy.logwarn('camera black script exit')
        return False  # stop listener
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    # if k in ['q']:  # keys of interest
    #     # self.keys.append(k)  # store it in global-like variable
    #     print('Key pressed: ' + k)
    #     return False  # stop listener; remove this if want more keys
    if k in ['a']:  # keys of interest
        # self.keys.append(k)  # store it in global-like variable
        if not b_set_loss:
            print('set to loss')
            rospy.loginfo('set camera black')
            b_set_loss = True
        else:
            print('set not to loss')
            rospy.loginfo('sonar recover')
            b_set_loss = False
        # return False  # stop listener; remove this if want more keys

if __name__ == '__main__':
    rospy.init_node('sonar_black')

    # set_camera_srv = rospy.Service('/mi_camera/set_camera_info', SetCameraInfo, set_camera_para)
    img_sub_l = rospy.Subscriber('/img_l/compressed', CompressedImage, camera_cb_l)
    img_sub_r = rospy.Subscriber('img_r/compressed', CompressedImage, camera_cb_r)

    # rospy.spin()

    listener = keyboard.Listener(on_press=on_press)
    listener.start()  # start to listen on a separate thread
    listener.join()  # remove if main thread is polling self.keys
