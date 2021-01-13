#!/usr/bin/env python

import rospy
import numpy as np

from   geometry_msgs.msg             import PoseArray, Pose
from   aruco_marker_localization.msg import MarkerPoseArray
from   std_msgs.msg                  import Int32


def callback(msg):
    poses = msg.poses
    ids = msg.ids
    if ids[0] == -1:
        rospy.loginfo('no markers detected')
    else:
        rospy.loginfo('{} marker(s) detected'.format(len(ids)))

def main():
    rospy.init_node('pose_subscriber_node', anonymous=True)
    rospy.Subscriber('/aruco_marker_ids_poses', MarkerPoseArray, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
