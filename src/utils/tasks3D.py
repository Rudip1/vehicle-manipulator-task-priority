#!/usr/bin/env python3
import numpy as np
import math
# from lab2_robotics imaport *
from utils.common_func import *
# from lab5 import * 

import sys
import os

# Get the directory of the current file (e.g., .../hoi/src/utils)
current_dir = os.path.dirname(os.path.abspath(__file__))

# Add to sys.path if not already added
if current_dir not in sys.path:
    sys.path.append(current_dir)

# Now import
from common_func import *


 
def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class Task:
    """
    Base class for task definition.

    Arguments:
    name (string): title of the task
    desired (Numpy array): desired task outcome (goal)
    """

    def __init__(self, name, desired):
        self.name = name # Name of the task
        self.sigma_d = desired # Desired outcome
        self.limit_activation = -1


    def update(self, robot):
        """Placeholder for method to update task variables."""
        pass

    def isActive(self):
        """Returns task activity status. Default is active."""
        return True 
    
    
    def setGain(self,K):
        self.K = K
    
    def getGain(self):
        return self.K
    
    
    def setDesired(self, value):
        """Sets the desired outcome."""
        self.sigma_d = value

    def getDesired(self):
        """Returns the desired outcome."""
        return self.sigma_d

    def getJacobian(self):
        """Returns the Jacobian of the task."""
        return self.J
    
    def get_base_jacobian(self):
        return
        
    def getJointPos(self, joint):
        
        return self.q[joint]

    def getError(self):
        """Returns the error of the task."""
        return self.err
    
    def Joint_limit_activation(self):
        """Returns the joint limit activation of the task."""
        return self.limit_activation
    
    def isTaskActive(self): 
        ''' check if the task is active '''
        if(self.limit_activation == 0):
            return False
        else:
            # print("joint limit activated")
            return True
        
    def get_link_Transform(self, link):
        return self.T[link]
    
    
    def getEEJacobian(self):
        return jacobian(self.T, self.revolute)[:2, :]

   
class Position3D(Task):
    """
    Subclass of Task, representing the 3D position task.
    """

    def __init__(self, name, desired, joint, K = np.diag([10,10,10])):
        super().__init__(name, desired)
        self.err = np.zeros((3,1))# Initialize error with proper dimensions 3 x 1
        self.J = np.zeros((3,6))
        self.joint = joint # The link for the task
        self.limit_activation = 1
        self.K =    K

    def update(self, robot):
        """Updates the task variables based on the current state of the robot."""
        self.J = robot.get_link_Jacobian(self.joint)[0:3]
        self.err = (self.getDesired() - robot.get_link_Transform(self.joint)[0:3, 3].reshape(3, 1)).reshape(3, 1)
        
class Orientation3D(Task):
    
    """
    Subclass of Task, representing the 3D orientation task.
    """

    def __init__(self, name, desired,joint=6, K = np.diag([10])):
        super().__init__(name, desired)
        self.err = np.zeros((1)) # Initialize error with proper dimensions
        self.J = np.zeros((1,6))
        self.joint = joint
        self.limit_activation = 1
        self.K = K

    def update(self, robot):
        """Updates the task variables based on the current state of the robot."""
        self.J = robot.get_link_Jacobian(self.joint)[-1].reshape(1, 6)
        angle = np.arctan2(robot.get_link_Transform(self.joint)[1,0], robot.get_link_Transform(self.joint)[0,0])
        self.err = self.sigma_d - np.array([[angle]]) # Update task error
        

class Base(Task):
    def __init__(self, name, desired, K = np.diag([5,5,5,2])):
        super().__init__(name, desired)
        
        self.activation_function = 1
        self.J = np.zeros((4,6))
        self.err = np.zeros((4,1))
        self.K = K 

    def update(self, robot):
        """Updates the task variables based on the current state of the robot."""
        self.J = robot.get_base_jacobian().reshape(4,6)
        
        sigma = np.zeros((4,1))
        
        sigma = robot.get_base_position().reshape(4,1)  
        
        # print("sigma", sigma) # 1X4
        # print("sigma_d", self.sigma_d)   # 4X1
        self.err = (self.sigma_d - sigma).reshape(4,1)
        
        # print("base_error", self.err)
    


class Configuration3D(Task):
    """
    Subclass of Task, representing the 3D configuration task.
    """

    def __init__(self, name, desired, joint=6, K = np.diag([5,5,5,0,0,2])):
        super().__init__(name, desired)
        self.joint = joint 
        self.activation_function = 1
        self.J = np.zeros((6, self.joint))
        self.err = np.zeros((6,1))
        self.K = K 

    def update(self, robot):
        """Updates the task variables based on the current state of the robot."""
        self.J = robot.get_link_Jacobian(self.joint).reshape(6,6)
        sigma = np.zeros((6,1))
        sigma[0:3] = robot.get_link_Transform(self.joint)[0:3, 3].reshape(3, 1)
       
        sigma[-1] = np.arctan2(robot.get_link_Transform(self.joint)[1,0], robot.get_link_Transform(self.joint)[0,0])
        # print("Sigma_D", sigma)
        self.err = (self.sigma_d - sigma).reshape(6,1)
        

class JointPosition(Task):
    """
    Subclass of Task, representing the joint position task.
    """
    def __init__(self, name, desired, joint, K = np.diag([10])):
        super().__init__(name, desired)
        self.joint = joint # The link for the task
        self.err = np.zeros((1,1))
        # self.J = np.zeros((1, 6))
        self.activation_function = 1
        self.K = K
        

    def update(self, robot):
        """Updates the task variables based on the current state of the robot."""
        self.J = np.zeros((1, robot.getDOF()))
        self.J[0, self.joint] = 1
        self.err = self.getDesired() - robot.getJointPos(self.joint)
        
        
class JointLimitTask(Task):
    def __init__(self, name, threshold, Q, joint, K =np.eye(1)):
        super().__init__(name, threshold)
        self.J = np.zeros((1, 6))
        self.err = np.ones((1,1))
        self.joint = joint
        self.alpha = threshold[0] 
        self.delta = threshold[1]
        self.qi_min = Q[0]
        self.qi_max = Q[1]
        self.limit_activation = -1
        self.K = K
        

    def update(self, robot):
        
        # print("self.limit_activation", self.J)
        
        # self.J = robot.get_link_Jacobian(self.joint)[5, :].reshape(1, robot.getDOF())
        self.J   =  np.zeros((1,robot.getDOF())) # jacobian
        self.J[0,self.joint-1] = 1
        
        joint_pose = robot.getJointPos(self.joint-1)
        
        
        # print("q",q)

        if (self.limit_activation == 0) and (joint_pose >= (self.qi_max - self.alpha)):
            self.limit_activation = -1
            
        elif (self.limit_activation == 0) and (joint_pose<= (self.qi_min + self.alpha)):
            self.limit_activation = 1
            
        elif (self.limit_activation == -1) and (joint_pose<= (self.qi_max - self.delta)):
            self.limit_activation = 0
            
        elif (self.limit_activation == 1) and (joint_pose >= (self.qi_min + self.delta)):
            self.limit_activation = 0
            
        self.err= np.array([self.limit_activation])
      
            
    