#!/usr/bin/env python

''' In this node an image is subscribed to and and it tries to detect
    ArUco markers and publishes a message which containts the marker ids
    and their corresponding pose information.
    When no marker is detected the published message will contain an invalide
    marker id, so it is easy to filter them. '''

# Importing all used packages
import rospy
import cv2
import cv2.aruco as aruco
import numpy     as np
# Importing all message types and converters
from   sensor_msgs.msg               import Image
from   geometry_msgs.msg             import PoseArray, Pose
from   aruco_marker_localization.msg import MarkerPoseArray
from   std_msgs.msg                  import Int32
# Specific functions from libraries
from   cv_bridge                 import CvBridge                                   # Function that converts Image.msg to cv2 image
from   tf.transformations        import quaternion_from_matrix                     # Function that converts 4x4 rotation matrix to quaternions

# Physical settings; marker size and camera matrix
# All markers used should have the same size!
marker_size              = rospy.get_param("/marker_size")                          # Length of one markers side in meters
camera_matrix            = np.array(rospy.get_param("/camera_matrix"))
camera_distortion_coeffs = np.array(rospy.get_param("/distortion_coefficients"))

# Setting for the aruco library; dictionary and detector parameters
dictionary = aruco.Dictionary_get(getattr(aruco, rospy.get_param("/dictionary")))
detector_parameters = aruco.DetectorParameters_create()

def rvecs_to_quats(rvecs):
    # Create a list to save quaternions for all detected markers
    qauts = []
    # Create a column and row to transform 3x3 matrix to 4x4 matrix
    add_column = np.array([0, 0, 0]).T
    add_row = np.array([0, 0, 0, 1])
    # Covert the rotation vector into quaternions
    for i in range(len(rvecs)):
        R_3x3 = cv2.Rodrigues(rvecs[i][0])[0]                                      # Convert rotation vector to 3x3 rotation matrix
        R_4x4 = np.row_stack((np.column_stack((R_3x3, add_column)), add_row))      # Convert 3x3 rotation matrix to 4x4 rotation matrix
        qaut  = quaternion_from_matrix(R_4x4)                                       # Convert 4x4 rotation matrix to quaternions
        qauts.append(qaut)
    return qauts


def create_message(qauts, tvecs, ids):
    # First create a list of Pose.msgs to publish in the MarkerPoseArray.msg
    poses = []                                                                     # Empty list for saving the poses
    pose_msg = Pose()                                                              # Creating Pose.msg
    # Create a list of the poses of all detected markers
    # By iterating over the amount of ids
    for i in range(len(tvecs)):
        pose_msg.position.x    = tvecs[i][0][0]
        pose_msg.position.y    = tvecs[i][0][1]
        pose_msg.position.z    = tvecs[i][0][2]
        pose_msg.orientation.x = qauts[i][0]
        pose_msg.orientation.y = qauts[i][1]
        pose_msg.orientation.z = qauts[i][2]
        pose_msg.orientation.w = qauts[i][3]
        poses.append(pose_msg)
    # Second create the MarkerPoseArray.msg to publish all marker poses with their
    # corresponding id, time stamp and coordinate frame
    poses_msg = MarkerPoseArray()                                                # Create MarkerPoseArray.msg
    poses_msg.poses = poses
    poses_msg.ids = ids
    poses_msg.header.stamp = rospy.Time.now()                                    # Create time stamp in seconds
    poses_msg.header.frame_id = '/camera'
    return poses_msg


def callback(msg):
    # Converting Image.msg to cv2 image
    br = CvBridge()
    image = br.imgmsg_to_cv2(msg)
    # Converting colour image to gray scale image
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Detecting markers in gray image, markers are detected from their
    # edges and the marker corners and ids outputted
    corners, ids, __ = aruco.detectMarkers(
        gray_image, dictionary, parameters=detector_parameters)
    # Checking if marker(s) are detected
    if ids is None:
        # No markers are detected
        rospy.loginfo('no markers detected')
        # Create false data when no marker is detected in the image
        ids   = np.array([[-1]])                                                  # -1 is an invalide marker id used to filter the data
        quats = np.array([[0, 0, 0, 0]])
        tvecs = np.array([[[0, 0, 0]]])
    elif not ids is None:
        # One or multiple markers are detected
        rospy.loginfo('{} marker(s) detected'.format(len(ids)))
        # Do the pose estimation for every marker detected
        rvecs, tvecs = aruco.estimatePoseSingleMarkers(
            corners, marker_size, camera_matrix, camera_distortion_coeffs)
        # Create the data for MarkerPoseArray.msg; quaternions(qx, qy, qz, qw)
        quats = rvecs_to_quats(rvecs)

    # Create MarkerPoseArray.msg and publish it as /aruco_marker_poses
    poses_msg = create_message(quats, tvecs, ids)
    pub       = rospy.Publisher('/marker_poses', MarkerPoseArray, queue_size=10)
    pub.publish(poses_msg)


def main():
    rospy.init_node('aruco_pose_estimation_node', anonymous=True)
    rospy.Subscriber('/image', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
