#!/usr/bin/env python

import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


cam1 = cv2.VideoCapture(1)
cam2 = cv2.VideoCapture(2)

bridge = CvBridge()

STEREO_FPS = 60

def talker():
    pub_left = rospy.Publisher('left_image_raw', Image, queue_size=STEREO_FPS)
    pub_right = rospy.Publisher('right_image_raw', Image, queue_size=STEREO_FPS)

    rospy.init_node('stereo', anonymous=True)
    rate = rospy.Rate(STEREO_FPS) # 10hz

    while not rospy.is_shutdown():
        hello_str = "Publishing at stereo/image_raw %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
            # Capture frame-by-frame
        ret1, frame1 = cam1.read()
        ret2, frame2 = cam2.read()

        # Our operations on the frame comes here
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        # Display the resulting frame
        #cv2.imshow('frame1',gray1)
        #cv2.imshow('frame2',gray2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        #pub.publish(hello_str)
	try:
            pub_left.publish(bridge.cv2_to_imgmsg(frame1, "bgr8"))
	    pub_right.publish(bridge.cv2_to_imgmsg(frame2, 'bgr8'))
        except CvBridgeError as e:
            print(e)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    cam1.release()
    cam2.release()
    cv2.destroyAllWindows()
