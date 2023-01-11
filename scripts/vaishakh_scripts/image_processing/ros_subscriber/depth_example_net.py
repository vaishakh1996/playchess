#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np


def convert_depth_image(ros_image):
    bridge = CvBridge()
    # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
     # Convert the depth image using the default passthrough encoding
        depth_image = bridge.imgmsg_to_cv2(
            ros_image, "passthrough")
    except CvBridgeError as e:
        print(e)
    # Convert the depth image to a Numpy array
    depth_array = np.array(depth_image, dtype=np.float32)
    print(depth_array)
    rospy.loginfo(depth_array)


def pixel2depth():
    rospy.init_node('pixel2depth', anonymous=True)
    rospy.Subscriber("/xtion/depth_registered/image_raw", Image,
                     callback=convert_depth_image, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    pixel2depth()
