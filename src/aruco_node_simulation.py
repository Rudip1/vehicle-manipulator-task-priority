#!/usr/bin/env python3

import roslib
import sys
import rospy
import tf2_ros
import cv2 as cv

import cv2.aruco as aruco
import numpy as np
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import tf
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation as R


class image_converter:

    def __init__(self):
        
        self.tfBuffer = tf2_ros.Buffer()
        
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
    
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        
        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber("turtlebot/kobuki/realsense/color/image_color", Image, self.callback)
        
        
        self.camera_info_sub = rospy.Subscriber("turtlebot/kobuki/realsense/color/camera_info", CameraInfo, self.camera_info_callback)
        self.odom_pub = rospy.Subscriber('/odom', Odometry, self.get_odom)
        #self.odom_pub = rospy.Subscriber('/turtlebot/kobuki/odom_ground_truth', Odometry, self.get_odom)

        
        self.measured_pub = rospy.Publisher("/aruco_position", PoseStamped , queue_size=1)
        
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)

        self.camera_id= None
        
        self.calibration_matrix = None
        self.distortion = None
        self.ids = None
        # self.marker_size_mymap = 0.06 # size of ArUco marker in meters
        self.aruco_size = 0.05 # size of ArUco marker in meters
        self.current_pose = None
        
    
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
        # print(f"current_pose {self.current_pose}")  
     
    def camera_info_callback(self, msg):
        
        self.camera_id = msg.header.frame_id
        
        # print(f"camera_id {self.camera_id}")
        self.calibration_matrix = np.array(msg.K).reshape((3, 3))
        # print("Camera_cal", self.calibration_matrix)
        self.distortion = np.array(msg.D)

    
    def callback(self, data):
        try:
            # print("Detecting ArUco markers...") 
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # print(f"cv_image {cv_image}")
        except CvBridgeError as e:
            print(e)

        # Detect the ArUco markers in the image
        # print(f"Detecting ArUco markers...", self.aruco_dict )
        corners, self.ids,_= cv.aruco.detectMarkers(cv_image, self.aruco_dict)

        if self.ids is not None:
            # Draw the detected markers and IDs on the image
            cv.aruco.drawDetectedMarkers(cv_image, corners, self.ids)

            # Get the position and orientation of each detected marker
            
            rot_vect, translation_vector, _ = cv.aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.calibration_matrix, self.distortion)
              
            # Draw the axes of each detected marker
            for i in range(len(self.ids)):
                # print(f"ids {self.ids}")
                cv.drawFrameAxes(cv_image, self.calibration_matrix,
                                self.distortion, rot_vect[i], 
                                translation_vector[i], self.aruco_size)
                        
                # Get the position and orientation of the marker
                rvec = rot_vect[i]
                tvec = translation_vector[i].reshape((-1,1))
                # print("rvect", rvec)
              
                pose_aruco = PoseStamped()
                pose_aruco.header.stamp = rospy.Time.now()
                pose_aruco.header.frame_id = self.camera_id
                
                # pose_aruco.header.stamp = rospy.Time.now()
                pose_aruco.pose.position.x = tvec[0]
                pose_aruco.pose.position.y = tvec[1]
                pose_aruco.pose.position.z = tvec[2]
     
                # Convert the rotation vector to a rotation matrix
                Rot, _ = cv.Rodrigues(rvec[0])
                
                Rotation = R.from_matrix(Rot)
                Quternion = Rotation.as_quat()
                pose_aruco.pose.orientation.x = Quternion[0]
                pose_aruco.pose.orientation.y = Quternion[1]
                pose_aruco.pose.orientation.z = Quternion[2]
                pose_aruco.pose.orientation.w = Quternion[3]
                
              
                roll_cam,pitch_cam, yaw_cam = np.pi/2.0, 0.0, np.pi/2.0
                
    
                # Define the rotation vector from the roll, pitch, and yaw angles
                rvec_c = np.array([roll_cam, pitch_cam, yaw_cam])
                
                R_c, _ = cv.Rodrigues(rvec_c)
                
                
                transform_robot_camera = self.tfBuffer.lookup_transform("world_ned", self.camera_id, rospy.Time(0), rospy.Duration(1.0))
                
            
                Position = do_transform_pose(pose_aruco, transform_robot_camera)
                
                self.measured_pub.publish(Position)
                
       
              
            resized = cv.resize(cv_image, (0, 0), fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)
            cv.imshow("Image window", resized)
            cv.waitKey(3)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)
                
            

def main(args):
    rospy.init_node('aruco_detection', anonymous=True)
    ic = image_converter()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)