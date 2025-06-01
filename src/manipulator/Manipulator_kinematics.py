#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped


class kinematics:
    def __init__(self):

        self.theta = None
        self.theta2 = None
        self.theta3 = None
        self.theta4 = None
        self.T1 = np.eye(4)
        self.T2 = np.eye(4)
        self.T3 = np.eye(4)
        self.T4 = np.eye(4)
        self.J = np.zeros((6, 4))
        
        self.transformation_mat = np.eye(4)
        
        self.pub = rospy.Publisher('/swiftpro/joint_velocity_controller/command', Float64MultiArray, queue_size=10)
        
        self.pose_stamped_pub = rospy.Publisher('/End_effector_pose', PoseStamped, queue_size=10)   
        

        self.joints_sub = rospy.Subscriber('/swiftpro/joint_states', JointState, self.JointState_callback)
        
        self.robot_pose = rospy.Subscriber('/turtlebot/odom', Odometry, self.get_odom)
        

    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y,odom.pose.pose.position.z, yaw])
   
        
        
        
        
    def kinematics_publisher(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            # Create a Float64MultiArray message object
            msg = Float64MultiArray()
            msg.data = [0, 0, 0, 0]

            self.pub.publish(msg)
            rate.sleep()

            

    def rot(self, axis, theta):
        # Generate rotation matrix around specified axis by theta (in radians)
        if axis == 'x':
            matrix = np.array([[1, 0, 0, 0],
                            [0, round(np.cos(theta), 2), -round(np.sin(theta), 2), 0],
                            [0, round(np.sin(theta), 2), round(np.cos(theta), 2), 0],
                            [0, 0, 0, 1]])
        elif axis == 'y':
            matrix = np.array([[round(np.cos(theta), 2), 0, round(np.sin(theta), 2), 0],
                            [0, 1, 0, 0],
                            [-round(np.sin(theta), 2), 0, round(np.cos(theta), 2), 0]
                            , [0, 0, 0, 1]])
        elif axis == 'z':
            matrix = np.array([[round(np.cos(theta), 2), -round(np.sin(theta), 2), 0, 0],
                            [round(np.sin(theta), 2), round(np.cos(theta), 2), 0, 0],
                            [0, 0, 1, 0], 
                            [0, 0, 0, 1]])
        else:
            raise ValueError("Invalid axis. Must be 'x', 'y', or 'z'.")
        matrix
        return matrix
    
    
    def translation_matrix(self, translation):
       
        if len(translation) != 3:
            raise ValueError("Invalid translation vector. Must have three elements.")
        
        matrix = np.eye(4)

        matrix[:3, 3] = translation
        return matrix


    def JointState_callback(self, data):

        # Check if the joint names in the received message match the expected names
        if data.name == ['swiftpro/joint1', 'swiftpro/joint2', 'swiftpro/joint3', 'swiftpro/joint4']:
            # Extract joint angles from the message
            self.theta, self.theta2, self.theta3, self.theta4 = data.position            
            self.x = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta)
            self.y = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.sin(self.theta)
            self.z = -0.108 -  0.142 * np.cos(self.theta2) - 0.1588 * np.sin(self.theta3)  + 0.0722
            
            
            self.transformation_mat= np.array([
                            [np.cos(self.theta+self.theta4),-np.sin(self.theta+self.theta4),0, self.x],
                            [np.sin(self.theta+self.theta4),np.cos(self.theta+self.theta4),0, self.y],
                            [0,0,1, self.z],
                            [0,0,0,  1    ]
                                          ]) 
            
      
            # print("EE position", self.x, self.y, self.z)
          
            self.marker_EE()


    
    
    def marker_EE(self):
        
        position_EE = self.transformation_mat[:, -1]
        pose_stamped = PoseStamped()
        pose_ = pose_stamped.pose.position   
        orient = pose_stamped.pose.orientation
        # Set the reference frame for the marker
        pose_stamped.header.frame_id = "swiftpro/manipulator_base_link"
        
        # Set the position of the pose based on the end effector position
        pose_.x = position_EE[0]
        pose_.y = position_EE[1]
        pose_.z = position_EE[2]

        # Set the orientation of the pose 
        q = quaternion_from_euler(0, 0, self.theta4)
        orient.x = q[0]
        orient.y = q[1]
        orient.z = q[2]
        orient.w = q[3]

        # Publish the PoseStamped to the specified topic
        self.pose_stamped_pub.publish(pose_stamped)
        




if __name__ == '__main__':
    try:
        rospy.init_node('kinematics_node', anonymous=True)
        K = kinematics()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


