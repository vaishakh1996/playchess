import rospy
from sensor_msgs.msg import Image
import ros_numpy


class DepthSubcriber():
    def __init__(self):
        self.sub = rospy.Subscriber(
            '/xtion/rgb/image_raw', Image, self.callback)
        self.depth_data_ = None

    def callback(self, msg):
        depth_data = ros_numpy.numpify(msg)
        print(depth_data.shape)
        self.depth_data_ = depth_data


if __name__ == '__main__':
    rospy.init_node('depth')

    depth_image = DepthSubcriber()

    try:
        while not rospy.is_shutdown():
            pass
    except KeyboardInterrupt:
        print('Shuting down DepthSubscriber')
