#!/user/bin/env python2 

import rospy

if __name__ == '__main__':
    rospy.init_node('test_node')
    rospy.loginfo('Test node has been Started')

    rate = rospy.Rate(10) # do somting 10 time per second

    # if ros has recived a shut down reques by ^c or other mean below false and node runs
    while not rospy.is_shutdown():
        rospy.loginfo('Hello')
        #run the loop at a rate described above
        rate.sleep()
    
