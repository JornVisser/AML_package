#!/usr/bin/env python

import rospy
from   sensor_msgs.msg import Image
from   cv_bridge       import CvBridge
import cv2

image_frequency = rospy.get_param("/image_frequency")

def publish_image(pub, rate):
    capture = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        [status, image] = capture.read()
        if status == True:
            rospy.loginfo('publish image')
            pub.publish(CvBridge().cv2_to_imgmsg(image))
        rate.sleep()

def main():
    rospy.init_node('image_publisher', anonymous=True)
    pub  = rospy.Publisher('/image', Image, queue_size=10)
    rate = rospy.Rate(image_frequency)
    publish_image(pub, rate)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
