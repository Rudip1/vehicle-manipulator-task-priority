#!/usr/bin/env python3  
import numpy as np    
import math            
         
class Task:
    '''
        Constructor.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired sigma (goal)
    '''
    def __init__(self, name, desired, activation=True):
        
        self.name = name # task title
        self.sigma_d = desired # desired sigma
        self.activation_function = True
        self.limit_activation = -1
                   
    '''
        Method updating the task variables (abstract).

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''
    def update(self, robot):
        pass

    '''
        Method updating activity of the task 

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''
    def isActive(self):
        return True 

    ''' 
        Method setting the desired sigma.

        Arguments:
        value(Numpy array): value of the desired sigma (goal)
    '''
    def setDesired(self, value):
        self.sigma_d = value

    '''
        Method returning the desired sigma.
    '''
    def getDesired(self):
        return self.sigma_d

    '''
        Method returning the task Jacobian.
    '''
    def getJacobian(self):
        return self.J

    '''
        Method returning the task error (tilde sigma).
    '''    
    def getError(self):
        return self.err
    
    def is_active(self):
        return self.activation_function
   
    def Joint_limit_activation(self):
        return self.limit_activation

'''
    Subclass of Task, representing the 3D position task.
'''
class Position3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired, activation=True)
        self.err = np.zeros((3,1))# Initialize with proper dimensions 3 x 1
        self.link = link 
        self.activation_function = True
        self.limit_activation = 0
               
    def update(self, robot):

        self.J = robot.get_link_Jacobian(self.link)[0:3]

        self.err = (self.getDesired() - robot.getLinkTransform(self.link)
                    [0:3, 3].reshape(3, 1)).reshape(3, 1)
        
        
'''
    Subclass of Task, representing the 3D orientation task.
'''
class Orientation3D(Task):
    def __init__(self, name, desired,link):
        super().__init__(name, desired, activation=True)
        self.err = np.zeros((1)) # Initialize with proper dimensions
        self.link = link 
        self.activation_function = True
        self.limit_activation = 1
        
        
    def update(self, robot):
        self.J = robot.get_link_Jacobian(self.link)[-1].reshape(1, 4)
        
        
        angle = np.arctan2(robot.getLinkTransform(self.link)[1,0], robot.getLinkTransform(self.link)[0,0])
        self.err = self.sigma_d - np.array([[angle]]) # Update task error
'''
    Subclass of Task, representing the 3D configuration task.
'''
class Configuration3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired, activation=True)
        self.link = link 
        self.activation_function = True


        #self.J = # Initialize with proper dimensions
        #self.err = # Initialize with proper dimensions
        
    def update(self, robot):
        self.J      = np.concatenate([robot.get_link_Jacobian(self.link)[0:3], robot.get_link_Jacobian(self.link)[-1].reshape(1, robot.getDOF())])
        sigma       = np.zeros((4,1))
        
        sigma[0:3] = robot.getLinkTransform(self.link)[0:3, 3].reshape(3, 1)

        sigma[3]    = np.arctan2(robot.getLinkTransform(self.link)[1,0], robot.getLinkTransform(self.link)[0,0])
        self.err    = (self.sigma_d - sigma).reshape(4,1)
        
'''
    Subclass of Task, representing the joint position task.
'''


class JointPosition(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired, activation=True)
        # self.J = # Initialize with proper dimensions  
        # self.err = # Initialize with proper dimensions
        self.link = link
       
    def update(self, robot):
        self.J = np.zeros((1, robot.getDOF()))
        self.J[0, self.link] = 1
        self.err = self.getDesired() - robot.getJointPos(self.link)        
        

class JointLimitTask(Task):
    def __init__(self, name, threshold, Q, link):
        super().__init__(name, threshold)
        self.link = link
        self.alpha = threshold[0]
        self.delta = threshold[1]
        self.qi_min = Q[0]
        self.qi_max = Q[1]
        self.limit_activation = -1

    def update(self, robot):
        self.J = robot.get_link_Jacobian(self.link)[5, :].reshape(1, robot.getDOF())
        self.err = 1
        q = math.atan2(robot.getLinkTransform(self.link)[1, 0], robot.getLinkTransform(self.link)[0, 0])
        self.joint_position = robot.getJointPos(self.link)

        if (self.limit_activation == 0) and (q >= (self.qi_max - self.alpha)):
            self.limit_activation = -1
            
        elif (self.limit_activation == 0) and (q <= (self.qi_min + self.alpha)):
            self.limit_activation = 1
            
        elif (self.limit_activation == -1) and (q <= (self.qi_max - self.delta)):
            self.limit_activation = 0
            
        elif (self.limit_activation == 1) and (q >= (self.qi_min + self.delta)):
            
            self.limit_activation = 0
            
            
    