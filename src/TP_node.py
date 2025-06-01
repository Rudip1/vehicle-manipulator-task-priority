#!/usr/bin/env python3

import rospy
import time
import numpy as np # Import Numpy
import math 

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped , Twist 
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion 
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String
from hoi.msg import CustomPoseStamped
import tf

from nav_msgs.msg import Odometry
from utils.tasks3D import *
from utils.common_func import *
from utils.lab6_robotics import *
# from utils.task_test import *   
import sys
import copy
import matplotlib.pyplot as plt

class manipulator():
    def __init__(self, theta, theta2, theta3, theta4):
        
         #### Base DH parameters ####
        self.DH_D    =  np.array([-0.198, 0.0507])     # displacement along Z-axis
        self.DH_theta =  np.array([-np.pi/2,0.0]).reshape(2,1) # rotation around Z-axis
        self.DH_alpha =  np.array([-np.pi/2,np.pi/2])                  # rotation around X-axis
        self.DH_a     =  np.zeros((2))  # displacement along X-axis
        self.revolute_Base = [True, False]        # flags specifying the type of joints
        self.J1_J2_Base_0 = np.zeros((6,2)) 
        self.goal_reached = False
        self.base_angle = 0.0 
        
        self.weight = np.array([[1,0,0,0,0,0],
                                [0,1,0,0,0,0],
                                [0,0,50,0,0,0],
                                [0,0,0,50,0,0],
                                [0,0,0,0,50,0], 
                                [0,0,0,0,0,50]])
        self.b = 0.235
        self.r = 0.035

        # Linear and angular speeds
        self.v = 0
        self.w = 0
        # Flags 
        self.left_wheel_received = False
        self.right_wheel_received = False
        # Wheel velocities
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
        self.last_time = rospy.Time.now()

        # Robot position initialization
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        
        self.dq_1 = []
        self.dq_2 = []
        self.dq_3 = []
        self.dq_4 = []
        self.dq_5 = []
        self.dq_6 = []
        
        self.position_err = []
        self.EE_x = []
        self.EE_y = []
        self.Base_x = []
        self.Base_y = [] 
        # self.join_vel = np.zeros((6,1))
                                 
        self.timesteps = []
        
        # print("dq", self.q_1,self.q_2, self.q_3, self.q_4, self.q_5, self.q_6)
        # Robot pose initialization
        self.xk = np.array([[self.x], [self.y], [self.th]])
        self.Pk = np.eye(3)*0  


        # Model noise initialization
        self.right_wheel_noise_sigma = 0.00000001
        self.left_wheel_noise_sigma = 0.00000001
        self.Qk = np.array([[self.right_wheel_noise_sigma**2, 0],
                            [0, self.left_wheel_noise_sigma**2]])
        ############################
        # self.current_pose = np.array([0,0,0,np.pi/4])
       
        self.revolute = [True, True, True, True]
        
        self.base = [True, False]
        self.base.extend(self.revolute) 
        
        self.dof = len(self.base)
        self.q = np.zeros(self.dof).reshape(-1, 1) # 6x1 
        self.theta, self.theta2, self.theta3, self.theta4 = theta, theta2, theta3, theta4
        self.i = 0
        self.T_ARMBASE_EE = np.eye((4)) 
       
        self.T_BASE_ARMBASE = np.eye((4))
        self.T_WORLD_BASE = np.eye((4))
        
        self.TB = kinematics(self.DH_D, self.DH_theta, self.DH_a, self.DH_alpha, self.T_WORLD_BASE)
        
        self.current_pose = [0, 0, 0, 0] 
        self.goal_cnt = 0
       
        self.vel_max_limit =0.2
        
        self.task_id = 0
        
        self.activation_threshold = [0.05,0.09]
        self.velo_limit = [-0.5, 0.5] # Joint velocity limits
    
        # Subs and Pubs
        self.odom_publisher= rospy.Publisher('/odom', Odometry, queue_size=10) #
        self.tf_br = tf.TransformBroadcaster()
        self.goal_sub=rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.task_callback)
        self.arm_joint_vel_pub = rospy.Publisher("/turtlebot/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10) # Manipulator only(q1_dot, q2_dot, q3_dot, q4_dot)
        self.robot_base_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) 
        self.goal_check = rospy.Publisher('/goal_Desired', PoseStamped, queue_size=10)
        self.marker_goal_pub = rospy.Publisher('goal_marker', Marker, queue_size=10) 
        self.EE_Pub = rospy.Publisher('/EndEffectorPose', PoseStamped, queue_size=10) 
        self.goal_reached_pub = rospy.Publisher('/task_feedback', String, queue_size=1 )
        # self.wheel_pub = rospy.Publisher('/turtlebot/kobuki/commands/wheel_velocities', Float64MultiArray, queue_size=10)
        # self.base_trajectory_marker_pub = rospy.Publisher('base_marker', Marker, queue_size=10)
        # self.endeffector_marker_pub = rospy.Publisher('endeffector_marker', Marker, queue_size=10)

        # self.goal_reached_sub = rospy.Subscriber('/goal', Bool, self.goal_reached_callback)
        rospy.Subscriber('/odom', Odometry, self.get_odom) 
        rospy.Subscriber('/turtlebot/joint_states', JointState, self.JointState_callback)
        rospy.Subscriber("/end_effector_pose", CustomPoseStamped, self.goal_callback)
        rospy.Subscriber('/turtlebot/kobuki/sensors/imu_data', Imu, self.imu_callback)
        rospy.Subscriber('/kobuki/kobuki/sensors/imu_data', Imu, self.imu_callback)
        rospy.Subscriber('/mobilebase/sensors/imu_data', Imu, self.imu_callback)
        # self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.twist_msg_callback)
        
        self.joint_limit_tasks= [   
                JointLimitTask("Joint limits",self.activation_threshold, [-1.571,1.571], 3),
                JointLimitTask("Joint limits",self.activation_threshold, [-1.571,0.05], 4),
                JointLimitTask("Joint limits",self.activation_threshold, [-1.571,0.05], 5),
                JointLimitTask("Joint limits",self.activation_threshold, [-1.571,1.571], 6)]

        # positions = [[1.0,3.5, -0.3], [1.0,3.5, -0.15], [1.0,3.5, -0.3], [3.0,3.5, -0.3], [3.0,3.5, -0.15], [3.0,3.5, -0.3]]
        # configurations = [[0.2,0.5, -0.1,0,0, math.pi/4], [0.2,0.5, -0.1,0,0, math.pi/3], [0.2,0.5, -0.1,0,0, math.pi/2]]
        #[0.2,0.5, -0.1,0,0, math.pi/4], [0.2,0.5, -0.1,0,0, math.pi/3], [0.2,0.5, -0.1,0,0, math.pi/2]] 
          
        # self.tasks2 = [self.joint_limit_tasks + [self.create_position_3d_task(pos, 6)] for pos in positions]
        # self.tasks2 = [self.joint_limit_tasks + [self.create_Configuration_3d_task(pos, 6)] for pos in configurations]  
        self.tasks2 = [self.joint_limit_tasks]
 
 
    def create_position_3d_task(self,position, joint):
            return Position3D("End-effector position", np.array(position).reshape(3, 1), joint)
  
    def create_Configuration_3d_task(self,position, joint):
            return Configuration3D("Configuration", np.array(position).reshape(6, 1), joint)

    def Base_task(self,position):
            return Base("Base", np.array(position).reshape(4, 1))
    
    def JointState_callback(self,data):
       
        if data.name[0] == 'turtlebot/kobuki/wheel_left_joint':
            self.left_wheel_vel = data.velocity[0]
            self.left_wheel_received = True
            
        elif data.name[0] == 'turtlebot/kobuki/wheel_right_joint':
            self.right_wheel_vel = data.velocity[0] #
            
            self.right_wheel_received = True
            
        if self.left_wheel_received and self.right_wheel_received:
            if data.name == ['turtlebot/swiftpro/joint1', 'turtlebot/swiftpro/joint2', 
                            'turtlebot/swiftpro/joint3', 'turtlebot/swiftpro/joint4']:
                
                # print("data_position:", data.position)
                self.theta, self.theta2, self.theta3, self.theta4 = data.position 
                
                self.update(self.theta, self.theta2, self.theta3, self.theta4) # 
                # print("manipulator_joints", data.position)
                left_lin_vel = self.left_wheel_vel * self.r
                right_lin_vel = self.right_wheel_vel * self.r

                self.v = (left_lin_vel + right_lin_vel) / 2.0
                self.w = (left_lin_vel - right_lin_vel) / self.b
                
                current_time = rospy.Time.from_sec(data.header.stamp.secs + data.header.stamp.nsecs * 1e-9)
                
                dt = (current_time - self.last_time).to_sec()
            
                self.last_time = current_time
                self.left_wheel_received = False
                self.right_wheel_received  = False
                ## 
                self.prediction(dt)
                # Odom path publisher
                self.odom_path_pub()
                                
                self.tasks = self.tasks2[self.i]
                # print("Task", self.tasks)
                self.TP()   
    
   ###########################
    #### TODO  NAVIGATION ####
    ###########################
    
    def wrap_angle(self, angle):
      
        return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )
    # Prediction ! 
    def prediction(self, dt):
    
        # Calculate Jacobians with respect to state vector
        Ak= np.array([[1, 0, -math.sin(float(self.xk[2]))*(self.v)*dt],
                      [0, 1, math.cos(float(self.xk[2]))*(self.v)*dt],
                      [0, 0, 1]])

        # Calculate Jacobians with respect to noise
        Wk = np.array([[0.5 * math.cos(float(self.xk[2]))*dt, 0.5 * math.cos(float(self.xk[2]))*dt],
                       [0.5 * np.sin(float(self.xk[2]))*dt, 0.5 *
                        math.sin(float(self.xk[2]))*dt],
                       [-dt/self.b, dt/self.b]])

        # Update the prediction "uncertainty"
        self.Pk = Ak @ self.Pk @ np.transpose(Ak) + Wk @ self.Qk @ np.transpose(Wk)

        # Integrate position
        self.xk[0] = self.xk[0] + self.v*math.cos(float(self.xk[2]))*dt
        self.xk[1] = self.xk[1] + self.v*math.sin(float(self.xk[2]))*dt
        self.xk[2] = wrap_angle(self.xk[2] + (self.w)*dt)
    
    ### Odom Publisher   
    def odom_path_pub(self):

        # Transform theta from euler to quaternion
        q = quaternion_from_euler(0, 0, float(wrap_angle(self.xk[2])))

        # Publish predicted odom
        odom = Odometry()
        
        odom.header.stamp = rospy.Time.now()
        
        odom.header.frame_id = "world_ned"
        odom.child_frame_id = "turtlebot/kobuki/base_footprint"

        odom.pose.pose.position.x = self.xk[0]
        odom.pose.pose.position.y = self.xk[1]

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
 
        odom.pose.covariance = [self.Pk[0, 0], self.Pk[0, 1], 0, 0, 0, self.Pk[0, 2],
                                self.Pk[1, 0], self.Pk[1,1], 0, 0, 0, self.Pk[1, 2],
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                self.Pk[2, 0], self.Pk[2, 1], 0, 0, 0, self.Pk[2, 2]]

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        self.odom_publisher.publish(odom)
     
        self.tf_br.sendTransform((float(self.xk[0]), float(self.xk[1]), 0.0), q, rospy.Time.now(), odom.child_frame_id, odom.header.frame_id)
    
    def imu_callback(self, msg):
        # print("IMU callback")   
        orientation = msg.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        self.yaw = np.array([yaw]).reshape(1, 1)
        
        self.heading_updae = True
        self.heading_update()
        
    def base_orientation_limit(self, angle):
        angle = np.clip(angle, -np.pi/4, np.pi/4)   
        return angle
    

    def heading_update(self):
        # print("Heading update")  
        # print("xk", self.xk) 
        # Create a row vector of zeros of size 1 x 3*num_poses
        self.compass_Vk = np.diag([0.00001])
        # define the covariance matrix of the compass
        self.compass_Rk = np.diag([0.00001]) 
        # print("imu update")   
        Hk = np.zeros((1, len(self.xk)))
        # Replace the last element of the row vector with 1
        Hk[0, -1] = 1
        predicted_compass_meas = self.xk[-1]
        # Compute the kalman gain
        K = self.Pk @ Hk.T @ np.linalg.inv((Hk @ self.Pk @ Hk.T) + (self.compass_Vk @ self.compass_Rk @ self.compass_Vk.T))

        # Compute the innovation
        innovation = np.array(self.wrap_angle(self.yaw[0] - predicted_compass_meas)).reshape(1, 1)

        # Update the state vector
        
        self.xk = self.xk + K@innovation
        
        # print("xk_updated", self.xk)    

        # Create the identity matrix        
        I = np.eye(len(self.xk))

        # Update the covariance matrix
        self.Pk = (I - K @ Hk) @ self.Pk @ (I - K @ Hk).T

    def goal_callback(self,msg_parent):
        self.id = msg_parent.id
        print("Task id", self.id)       
        msg = msg_parent.pose
        
        goal_reached_msg = PoseStamped()
        msg.header.frame_id = "world_ned"
        
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
    
        _,_,yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, 
                        msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
       
       
        # print("Task", task) 
        
        if self.id == 2:
            task_ = [x, y, z, yaw]
            self.weight = np.array([[1,0,0,0,0,0],
                                    [0,1,0,0,0,0],
                                    [0,0,10000,0,0,0],
                                    [0,0,0,10000,0,0],
                                    [0,0,0,0,10000,0], 
                                    [0,0,0,0,0,5000]])
            task = self.Base_task(task_)
            
        # arm only task 
        # elif self.id == 11:
        #     task_ = [x, y, z]
            
        #     self.weight = np.array([[500,0,0,0,0,0],
        #                             [0,50,0,0,0,0],
        #                             [0,0,50,0,0,0],
        #                             [0,0,0,50,0,0],
        #                             [0,0,0,0,50,0], 
        #                             [0,0,0,0,0,1]])
            
        #     task = self.create_position_3d_task(task_, 6)
        
        elif self.id == 11:
            task_ = [x, y, z]
            
            self.weight = np.array([[1000,0,0,0,0,0],
                                    [0,200,0,0,0,0],
                                    [0,0,10,0,0,0],
                                    [0,0,0,10,0,0],
                                    [0,0,0,0,10,0], 
                                    [0,0,0,0,0,1]])
            
            task = self.create_position_3d_task(task_, 6)
            

        elif self.id == 3:
            task = JointPosition("Joint position", np.array([x]).reshape(1, 1),int(y))
            
        elif self.id == 4:
            self.weight = np.array([[50,0,0,0,0,0],
                                    [0,50,0,0,0,0],
                                    [0,0,10,0,0,0],
                                    [0,0,0,10,0,0],
                                    [0,0,0,0,20,0], 
                                    [0,0,0,0,0,1]])
            
            task = self.create_Configuration_3d_task([x, y, z, 0, 0, yaw], 6)   
            
        self.tasks2 = [self.joint_limit_tasks + [task]]
 
    def task_callback(self,msg):
        self.i = self.i+1
        
    #Send command_velocity to the mobile base manipulator    
    def send_vel(self, q): # q = [w, v, q1_dot, q2_dot, q3_dot, q4_dot] 
        p = Float64MultiArray()
        p.data = [float(q[2]), float(q[3]), float(q[4]), float(q[5])]
        base_vel = Twist()
        base_vel.linear.x = q[1]
        base_vel.linear.y = 0
        base_vel.linear.z = 0
        base_vel.angular.x = 0
        base_vel.angular.y = 0
        base_vel.angular.z = q[0]  # Set base velocities
        self.arm_joint_vel_pub.publish(p)  # Publish joint velocity
        self.robot_base_vel_pub.publish(base_vel)  # Publish base velocity
                         
    def TP(self):
        
        pose_stamp = PoseStamped()
        pose_stamp.header.frame_id = "world_ned"
        goal_reached_msg = Bool()
        tasks = self.tasks
        P = np.eye(6)  # Initialize the pseudo-inverse matrix P
        dq = np.zeros((6, 1))  # Initialize the joint velocity vector dq
     
        for i in range(len(tasks)):
            tasks[i].update(self)  # Update the task
            aug_Jacobian = tasks[i].getJacobian() @ P  # Calculate the augmented Jacobian matrix
            
            if tasks[i].isTaskActive():  # Check if the task is active
                gain = tasks[i].getGain()
                dq = dq + self.DLS(aug_Jacobian, 0.05, self.weight) @ (gain@tasks[i].getError() - (tasks[i].getJacobian() @ dq))  
                P = P - np.linalg.pinv(aug_Jacobian) @ aug_Jacobian  # Update the pseudo-inverse matrix P
            else:
                dq = dq
                P = P  
                
            if (tasks[i].name == "Base"):             
                if (np.linalg.norm(tasks[i].getError()))< 0.03:
                    self.goal_reached = True
                    check_goal = String()
                    check_goal.data = "success"
                    self.goal_reached_pub.publish(check_goal)
                    
            elif tasks[i].name == "End-effector position" or tasks[i].name == "Configuration":
                if (np.linalg.norm(tasks[i].getError())) < 0.03:
                    self.goal_reached = True
                    check_goal = String()
                    check_goal.data = "success"
                    self.goal_reached_pub.publish(check_goal)
            
        dq_scaled = self.vel_limit(dq, self.vel_max_limit) 
        
        if tasks[i].name != "Joint limits":
            self.dq_1.append(dq_scaled[0])
            self.dq_2.append(dq_scaled[1])
            self.dq_3.append(dq_scaled[2])
            self.dq_4.append(dq_scaled[3])
            self.dq_5.append(dq_scaled[4])
            self.dq_6.append(dq_scaled[5])
            
            self.EE_x.append(self.kinematics()[0,3])
            self.EE_y.append(self.kinematics()[1,3])
            self.Base_x.append(self.current_pose[0])
            self.Base_y.append(self.current_pose[1])      

            self.timesteps.append(self.timesteps[-1] + 0.017) if self.timesteps else self.timesteps.append(0.0)
            self.position_err.append(np.linalg.norm(tasks[i].getError()))
            # self.join_vel = np.append(self.join_vel, dq_scaled)

        self.send_vel(dq_scaled)  # Send joint velocities to the robot
        
         #### EE POSE AND DESIRED PUblisher.
        pose_ee = self.getEETransform()
        ee_angle = self.getEEOrientation()
        # Publish Current End-Effector Position
        ee_pose = PoseStamped()
        ee_pose.header.stamp= rospy.Time.now()
        ee_pose.header.frame_id = 'world_ned'
        # ee_pose.header.frame_id = 'turtlebot/kobuki/base_footprint'
        ee_pose.pose.position.x = pose_ee[0]
        ee_pose.pose.position.y = pose_ee[1]
        ee_pose.pose.position.z = pose_ee[2]
        
        # Set the orientation of the end effector
        quaternion = tf.transformations.quaternion_from_euler(0, 0, ee_angle)
        ee_pose.pose.orientation.x = quaternion[0]
        ee_pose.pose.orientation.y = quaternion[1]
        ee_pose.pose.orientation.z = quaternion[2]
        ee_pose.pose.orientation.w = quaternion[3]
        self.EE_Pub.publish(ee_pose)
       
        # Publish the Desired End-Effector Position
        desired_pose = PoseStamped()
        desired_pose.header.stamp= rospy.Time.now()
        desired_pose.header.frame_id = 'world_ned'
        
        # ee_pose.header.frame_id = 'turtlebot/kobuki/base_footprint'
        if tasks[i].name == "Configuration":
            desired_pose.pose.position.x = tasks[i].getDesired()[0]
            desired_pose.pose.position.y = tasks[i].getDesired()[1]
            desired_pose.pose.position.z = tasks[i].getDesired()[2]
            
            # Set the desired orientation of the end effector
            yaw =copy.copy(tasks[4].getDesired()[-1])
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            desired_pose.pose.orientation.x = quaternion[0]
            desired_pose.pose.orientation.y = quaternion[1]
            desired_pose.pose.orientation.z = quaternion[2]
            desired_pose.pose.orientation.w = quaternion[3]
            self.goal_check.publish(desired_pose)   
            
        if tasks[i].name == "End-effector position":
            desired_pose.pose.position.x = tasks[i].getDesired()[0]
            desired_pose.pose.position.y = tasks[i].getDesired()[1]
            desired_pose.pose.position.z = tasks[i].getDesired()[2]
            
            # Set the desired orientation of the end effector
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
            desired_pose.pose.orientation.x = quaternion[0]
            desired_pose.pose.orientation.y = quaternion[1]
            desired_pose.pose.orientation.z = quaternion[2]
            desired_pose.pose.orientation.w = quaternion[3]
            self.goal_check.publish(desired_pose) 
        
        
        if self.goal_reached == True:
            self.goal_reached = False 
            self.tasks2 = [self.joint_limit_tasks ]
            # self.goal_reached = True
            
    # Current position of the robot from the odometry
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, yaw])
        self.base_angle = self.current_pose[3]
        self.B_Transform()
                
    def goal_reached_callback(self, msg):
        return
    # msg, bool
    #   self.goal_reached = msg.data 
 
    # Forward kinematics   
    def kinematics(self): 
        
        # Geometric end-effector position with respect to the manipulator base 
        self.x = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta)
        self.y = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.sin(self.theta)
        self.z = -0.108 -  0.142 * np.cos(self.theta2) - 0.1588 * np.sin(self.theta3)  + 0.0722
            
        self.T_ARMBASE_EE=np.array([[np.cos(self.theta+self.theta4),-np.sin(self.theta+self.theta4),0, self.x],
                                                 [np.sin(self.theta+self.theta4),np.cos(self.theta+self.theta4),0, self.y],
                                                 [0,0,1, self.z],
                                                 [0,0,0,  1    ]])  
                            
        
        self.T_BASE_ARMBASE = np.array([
                            [math.cos(-math.pi/2) , -math.sin(-math.pi/2), 0, 0.051],
                            [math.sin(-math.pi/2), math.cos(-math.pi/2), 0, 0],
                            [0             ,  0               , 1 , -0.198],
                            [0             ,  0               , 0, 1]])
          
        self.T_WORLD_BASE = np.array([
                            [math.cos(self.current_pose[3]) , -math.sin(self.current_pose[3]), 0, self.current_pose[0]],
                            [math.sin(self.current_pose[3]), math.cos(self.current_pose[3]), 0, self.current_pose[1]],
                            [0             ,  0               , 1 , self.current_pose[2]],
                            [0             ,  0               , 0, 1]])
        
        
        # Modify the world-to-robot transformation matrix by applying the rotation
        self.T = self.T_WORLD_BASE @ self.T_BASE_ARMBASE @ self.T_ARMBASE_EE  
      
        return self.T
    
    def B_Transform(self):
        self.T_WORLD_BASE = np.array([
                                [math.cos(self.current_pose[3]) , -math.sin(self.current_pose[3]), 0, self.current_pose[0]],
                                [math.sin(self.current_pose[3]), math.cos(self.current_pose[3]), 0, self.current_pose[1]],
                                [0             ,  0               , 1 , self.current_pose[2]],
                                [0             ,  0               , 0, 1]])
                    
        
        self.TB  = kinematics(self.DH_D, self.DH_theta, self.DH_a, self.DH_alpha, self.T_WORLD_BASE)
        # self.EE_POSE_Pub()
        return self.TB
    
    def Jacobian_B(self):
            
        Tran= self.kinematics()
        
        Po_EE = Tran[:3,3]  
        
        Base_Jacobain  = jacobian(self.B_Transform(),Po_EE.reshape(-1,1), self.revolute_Base)   
        # print(Base_Jacobain)
        # print("Base_Jacobain", Base_Jacobain[:,0])
        return Base_Jacobain.reshape(6,2) # 6x2
            
      
    def Jacobian(self):
        
        # World_EE_Transformation Jacobian     
        # partial derivative of x, y, z with respect to q1
        EEx_theta1 = -0.0565*np.sin(self.theta)*math.sin(self.current_pose[3]) + 0.0565*np.cos(self.theta)*(math.cos(self.current_pose[3])-math.sin(self.current_pose[3]))
        EEy_theta1 = ((0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta))*(math.cos(self.current_pose[3])+math.sin(self.current_pose[3]))\
            +((0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta))*(math.sin(self.current_pose[3]))
        EEz_theta1 = 0
        
        # partial derivative of x, y, z with respect to q2
        EEx_theta2 = -0.142*np.cos(self.theta2)*np.cos(self.theta)*math.sin(self.current_pose[3]) - 0.142*np.cos(self.theta2)*np.sin(self.theta)*(math.cos(self.current_pose[3])-math.sin(self.current_pose[3]))
        EEy_theta2 = -0.142*np.cos(self.theta2)*np.sin(self.theta)*(math.cos(self.current_pose[3])+math.sin(self.current_pose[3]))+0.142*np.cos(self.theta2)*np.cos(self.theta)*math.cos(self.current_pose[3])
        EEz_theta2 = 0.142 * np.sin(self.theta2)
        
        # partial derivative of x, y, z with respect to q3
        EEx_theta3 = -0.1588*np.sin(self.theta3)*np.cos(self.theta)*math.sin(self.current_pose[3]) - 0.1588*np.sin(self.theta3)*np.sin(self.theta)*(math.cos(self.current_pose[3])-math.sin(self.current_pose[3]))
        EEy_theta3 = -0.1588 * np.sin(self.theta3) * np.sin(self.theta)*(math.cos(self.current_pose[3])+math.sin(self.current_pose[3])) + 0.1588*np.sin(self.theta3) * np.cos(self.theta)*math.cos(self.current_pose[3])
        EEz_theta3 = -0.1588 * np.cos(self.theta3) 
        
        # partial derivative of x, y, z with respect to q4
        EEx_theta4 = 0
        EEy_theta4 = 0
        EEz_theta4 = 0
        
        self.J1_J2_Base_0[:,0] = self.Jacobian_B()[:,0]
    
        # print("reshapeeee..", self.Jacobian_B()[:,1].reshape(6,1))
        self.J1_J2_Base_0[:,1] = self.Jacobian_B()[:,1]
        
        # print("___",self.J1_J2_Base_0.shape)
        # J1 = self.J1_J2_Base_0[:,0]
        # J2 = self.J1_J2_Base_0[:,1]
        
        J3 = np.array([EEx_theta1, EEy_theta1, EEz_theta1, 0, 0, 1])
        J4 = np.array([EEx_theta2, EEy_theta2, EEz_theta2, 0, 0, 0])
        J5 = np.array([EEx_theta3, EEy_theta3, EEz_theta3, 0, 0, 0])
        J6 = np.array([EEx_theta4, EEy_theta4, EEz_theta4, 0, 0, 1])     
        self.J = np.column_stack((self.J1_J2_Base_0, J3, J4, J5, J6))
             
        return self.J
       
     # Scale the joint velocities to ensure that the maximum velocity is not exceeded  
    def vel_limit(self, dq, dq_max):
        # Calculate the absolute ratio of each velocity to its maximum limit
        ratios = np.abs(dq / dq_max)

        # Find the maximum of these ratios
        s = np.max(ratios)

        if s > 1:
            dq_scaled = dq / s
        else:
            dq_scaled = dq

        return dq_scaled
    
    
    # Update the manipulator joint angles
    def update(self, theta, theta2, theta3, theta4):
        
        self.kinematics()
        self.Jacobian()
        
        self.theta = theta 
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.q = [self.base_angle, 0, self.theta, self.theta2, self.theta3, self.theta4]
        
    def get_base_jacobian(self):
        
        jacob = self.Jacobian_B()
        jacob_1 = jacob[:3,:]
        jacob_2 = jacob[-1,:]
        jacob_new = np.vstack((jacob_1, jacob_2))
             
        base_jacob = np.zeros((4,4))
        
        Base_jacobian = np.block([jacob_new, base_jacob])
        # print("Base_jacobian", Base_jacobian)
        return Base_jacobian
    
     
    def get_base_position(self): # x,y,z,theta
        return self.current_pose

    '''
        Method that returns the end-effector Jacobian.
    '''
    def getEEJacobian(self, link):
        return self.Jacobian()[:, link] 
    
    def getJacobian(self):
        return self.J
    '''
        Method that returns the end-effector transformation.
    '''
    def getEETransform(self):
        return self.kinematics()[:3,3]
    
    def getEEOrientation(self):
        return np.arctan2(self.kinematics()[1,0],self.kinematics()[0,0])
    
    def get_link_Jacobian(self, link):
        return self.Jacobian() #[:, link]

    '''
        Method that returns the orientation jacobian Jacobian.
    '''
    def getOrientationJacobian(self, link):
        return self.get_link_Jacobian(link)[5, :] # Orientation Jacobian
    
   
    def get_link_Transform(self,joint=6):
            return self.kinematics()
    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
        return self.dof
    
    
    def getJointPos(self, joint):
        return self.q[joint]
                                     
    def DLS(self, A, damping, weights):
                
        A_TA = (A @ np.linalg.inv(weights)) @ A.T
        I = np.identity(A_TA.shape[0])
        DLS = np.linalg.inv(A_TA + damping**2 * I)
        DLS = np.linalg.inv(weights) @ A.T @ DLS
        
        return  DLS

    # Desired position of the end-effector 
    def goal_check_marker(self,g):
        
        # print("goal marker")
        marker = Marker()
        marker.header.frame_id = "world_ned"
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = g[0]
        marker.pose.position.y = g[1]
        marker.pose.position.z = g[2]

        # marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.g = 0.9
        marker.color.r = 0.0
        marker.color.a = 0.8

        self.marker_goal_pub.publish(marker)
if __name__ == '__main__':
    try:
        rospy.init_node('Task_priority_node', anonymous=True)
        task = manipulator(0.0, 0.0, 0.0, 0.0)
        # a.EE_POSE_Pub()
        rospy.spin()  
        
        
                
         # Plot end-effector and base positions
        plt.figure()
        plt.plot(task.EE_x, task.EE_y, label='End-Effector Position', color="red")
        plt.plot(task.Base_x, task.Base_y, label='Base Position', color="green")
        plt.xlim([0, max(task.EE_x)])
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title('Base EE-Effector position for pick and place')
        plt.grid()
        plt.legend()
        # plt.savefig('TP_c_config_Position_EE_B.png')
        # print("Saved: TP_c_config_Position_EE_B.png")
        plt.show()

        # Plot position error over time
        plt.figure()
        plt.plot(task.timesteps, task.position_err, label='e1 (End-Effector Error)', color="red")
        plt.xlim([0, max(task.timesteps)])
        plt.xlabel('Time (s)')
        plt.ylabel('Error (m)')
        plt.title('TP Control Error for Pick and place task')
        plt.grid()
        plt.legend()
        plt.savefig('TP_c_config_error.png')
        print("Saved: TP_c_config_error.png")
        plt.show()

        plt.figure()
        plt.plot(task.timesteps, task.dq_1, label='dq1', color="red")
        plt.plot(task.timesteps, task.dq_2, label='dq2', color="blue")
        plt.plot(task.timesteps, task.dq_3, label='dq3', color="green")
        plt.plot(task.timesteps, task.dq_4, label='dq4', color="black")
        plt.plot(task.timesteps, task.dq_5, label='dq5', color="orange")
        plt.plot(task.timesteps, task.dq_6, label='dq6', color="purple")
        plt.xlim([0, max(task.timesteps)])
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity')
        plt.title('Joint Velocities for Pick and Place task')
        plt.grid()
        plt.legend()
        # plt.savefig('TP_c_config_joint_vel.png')
        # print("Saved: TP_c_config_joint_vel.png")
        plt.show()
        sys.exit(0)

    except rospy.ROSInterruptException:
        pass 