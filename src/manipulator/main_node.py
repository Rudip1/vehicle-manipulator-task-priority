#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped , Twist 
import time 
import numpy as np 
import math  
from tf.transformations import quaternion_from_euler
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from Manipulator_task import Position3D, Orientation3D, Configuration3D, JointPosition, JointLimitTask

class Manipulator():
    def __init__(self, theta, theta2, theta3, theta4):
        self.revolute = [True, True , True , True]
        self.dof = len(self.revolute)
        self.q = np.zeros(self.dof).reshape(-1, 1)
        self.theta = theta
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
         
        self.ee_position = [0, 0, 0, 0]
        self.ee_path_pub = rospy.Publisher('/end_effector_path', Path, queue_size=10)
        self.ee_path = Path()
        self.ee_path.header.frame_id = "swiftpro/manipulator_base_link"
        
        self.joints_sub = rospy.Subscriber('/swiftpro/joint_states', JointState, self.JointState_callback)
        self.T_end_effector= np.zeros((4,4))
        
        self.joint_velocity= rospy.Publisher("/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10)
        
        self.goal_check = rospy.Publisher('/Goal_Pose', PoseStamped, queue_size=10)   
        time.sleep(1)  
        

       
    
    def JointState_callback(self,data):
        if data.name == ['swiftpro/joint1', 'swiftpro/joint2', 'swiftpro/joint3', 'swiftpro/joint4']:
            # Extract joint angles from the message
            self.theta, self.theta2, self.theta3, self.theta4 = data.position 
            self.update(self.theta, self.theta2, self.theta3, self.theta4)
            _, x, y, z = self.kinematics()
            self.update_ee_path(x, y, z)
            goals = [[0, [0.13, 0.3, -0.15]]] # [position] Position 3d check  
            #goals_list = [[0, [0.11, 0.3, -0.15]],[0, [0.09, 0.3, -0.15]],[0, [0.10, 0.3, -0.15]],[2, [0.11, 0.3, -0.15, np.pi/2]] ] # [position] Position 3d check
            #for i in range(len(goals_list)):
                #    print("Goal", goals_list[i])
                 #   goals = goals_list[i]
            self.Task_priority_algorithm(goals)
            
    def kinematics(self): 
        # Geometric end-effector position with respect to the manipulator base 
        self.x = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta)
        self.y = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.sin(self.theta)
        self.z = -0.108 -  0.142 * np.cos(self.theta2) - 0.1588 * np.sin(self.theta3)  + 0.0722
            
            
        self.trans_mat= np.array([
                            [1,0,0, self.x],
                            [0,1,0, self.y],
                            [0,0,1, self.z],
                            [0,0,0,  1    ]
                                          ]) 
        return self.trans_mat, self.x, self.y, self.z
    
    
    def Jacobian(self):
         # partial derivative of the end-effector position with respect to the joint angles
        EEx_dq1 = -np.sin(self.theta) * (0.0697 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3))
        EEx_dq2 = -0.142 * np.cos(self.theta) * np.cos(self.theta2)
        EEx_dq3 = -0.1588 * np.sin(self.theta3) * np.cos(self.theta)
        EEx_dq4 = 0        
        EEy_dq1 = np.cos(self.theta) * (0.0697 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3))
        EEy_dq2 = -0.142 * np.sin(self.theta) * np.cos(self.theta2)
        EEy_dq3 = -0.1588 * np.sin(self.theta3) * np.sin(self.theta)
        EEy_dq4 = 0
        
        EEz_dq1 = 0
        EEz_dq2 = 0.142 * np.sin(self.theta2)
        EEz_dq3 = -0.1588 * np.cos(self.theta3) 
        EEz_dq4 = 0
        
        self.J = np.array([ 
                            [EEx_dq1 , EEx_dq2  ,EEx_dq3  ,EEx_dq4],
                            [EEy_dq1 , EEy_dq2  ,EEy_dq3  ,EEy_dq4],
                            [EEz_dq1 , EEz_dq2  ,EEz_dq3  ,EEz_dq4],
                            [ 0    , 0     ,   0   , 0 ],
                            [ 0    , 0     ,   0   , 0 ],
                            [1    , 0     ,   0   , 1 ]
                                                                           ])
        return self.J
    
    
    def update(self, theta, theta2, theta3, theta4):
        self.theta = theta
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.q = [self.theta, self.theta2, self.theta3, self.theta4]
        

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
        return self.trans_mat
    
    def get_link_Jacobian(self, link):
        return self.Jacobian()#[:, link]

    def getLinkTransform(self,link):
            return self.kinematics()[0]
    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
        return self.dof
    
    def getJointPos(self, joint):
        return self.q[joint]
                                                        
    def DLS(self, A, damping):
      
        A_TA = np.matmul(A.T, A)
        I = np.identity(A_TA.shape[0])
        DLS = np.linalg.inv(A_TA + damping**2 * I)
        DLS = np.matmul(DLS, A.T)
        # Implement the formula to compute the DLS of matrix A.
        return  DLS
    
    def send_commnd(self, q):
        # Manipulator base publisher 
        p = Float64MultiArray()
        p.data = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        # print("Arm Joint velocity", p.data)
        self.joint_velocity.publish(p)

    # Task definition                                          
    def tasks(self, goals):
        tasks = []
        pose_stamped = PoseStamped()
        pose_ = pose_stamped.pose.position
        orient = pose_stamped.pose.orientation
        pose_stamped.header.frame_id = "swiftpro/manipulator_base_link"

        for goal in goals:
            if goal[0] == 0:
                tasks.append(Position3D("End-effector position", np.array(goal[1]).reshape(3,1), link=3))                
                # Set the position of the pose based on the end effector position
                pose_.x = goal[1][0]
                pose_.y = goal[1][1]
                pose_.z = goal[1][2]

            elif goal[0] == 1:
                tasks.append(Orientation3D("end-effector orientation", np.array(goal[1]), link=3))
                q = quaternion_from_euler(0, 0, goal[1][0])
                orient.x = q[0]
                orient.y = q[1]
                orient.z = q[2]
                orient.w = q[3]
                
            elif goal[0] == 2:
                tasks.append(Configuration3D("end-effector configuration", np.array(goal[1]).reshape(4, 1),  link=3))
                pose_.x = goal[1][0]
                pose_.y = goal[1][1]
                pose_.z = goal[1][2]
                q = quaternion_from_euler(0, 0, goal[1][3])
                orient.x = q[0]
                orient.y = q[1]
                orient.z = q[2]
                orient.w = q[3]
                
            elif goal[0] == 3:
                tasks.append(JointPosition("joint 1 position", np.array(goal[1][0]), link = goal[1][1]))
                
            elif goal[0] == 4:
                tasks.append(JointLimitTask("Joint 1 limit", np.array(goal[1][0]), np.array(goal[1][1]), link = goal[1][2]))


        self.goal_check.publish(pose_stamped)
        return tasks
    
    def Task_priority_algorithm(self, goal):
        tasks = self.tasks(goal)            
        dof = self.getDOF()
        # # Initialize null-space projector
        P = np.eye(dof)
        # # Initialize output vector (joint velocity)
        dq = np.zeros((dof, 1))
        P = np.eye(self.getDOF())
        # Initialize output vector (joint velocity)
        dq = np.zeros((self.getDOF(),1)) 
       
        for i in range(len(tasks)):

            tasks[i].update(self)
            # Compute augmented Jacobian
            aug_Jacobian = tasks[i].getJacobian() @ P

            # Compute task velocity
            dq = dq + self.DLS(aug_Jacobian, 0.1) @ (tasks[i].getError() - (tasks[i].getJacobian() @ dq))
            P = P - np.linalg.pinv(aug_Jacobian) @ aug_Jacobian
        if np.linalg.norm(tasks[0].getError()) < 0.005:
            rospy.loginfo(" Goal reached. Stopping movement.")
            self.send_commnd([0.0, 0.0, 0.0, 0.0])
            return
        self.send_commnd(dq)
    def update_ee_path(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "swiftpro/manipulator_base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0  # Neutral orientation
        self.ee_path.poses.append(pose)
        self.ee_path_pub.publish(self.ee_path)

                    
        
if __name__ == '__main__':
    try:
        rospy.init_node('Task_priority_node', anonymous=True)
        task= Manipulator(0, 0, 0, 0)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass