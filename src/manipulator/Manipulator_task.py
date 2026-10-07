#!/usr/bin/env python3
"""Task definitions for the arm-only (4-DOF uArm Swift Pro) task-priority controller."""
import numpy as np
import math

class Task:
    '''
        Base class representing an abstract task.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired sigma (goal)
        activation (bool): unused; kept for interface compatibility
    '''
    def __init__(self, name, desired, activation=True):

        self.name = name # task title
        self.sigma_d = desired # desired sigma
        self.activation_function = True
        self.limit_activation = -1

    def update(self, robot):
        '''
            Method updating the task variables (abstract).

            Arguments:
            robot (object of class Manipulator): reference to the manipulator
        '''
        pass

    def isActive(self):
        '''
            Method returning the activity of the task (always active).
        '''
        return True

    def setDesired(self, value):
        '''
            Method setting the desired sigma.

            Arguments:
            value(Numpy array): value of the desired sigma (goal)
        '''
        self.sigma_d = value

    def getDesired(self):
        '''
            Method returning the desired sigma.
        '''
        return self.sigma_d

    def getJacobian(self):
        '''
            Method returning the task Jacobian.
        '''
        return self.J

    def getError(self):
        '''
            Method returning the task error (tilde sigma).
        '''
        return self.err

    def is_active(self):
        '''
            Method returning the activation flag of the task.
        '''
        return self.activation_function

    def Joint_limit_activation(self):
        '''
            Method returning the joint-limit activation value (-1, 0 or 1).
        '''
        return self.limit_activation

class Position3D(Task):
    '''
        Subclass of Task, representing the 3D position task.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired end-effector position (3 x 1)
        link (integer): link index passed to the robot model
    '''
    def __init__(self, name, desired, link):
        super().__init__(name, desired, activation=True)
        self.err = np.zeros((3,1))# Initialize with proper dimensions 3 x 1
        self.link = link
        self.activation_function = True
        self.limit_activation = 0

    def update(self, robot):
        '''
            Method updating the task Jacobian and error from the robot state.
        '''

        self.J = robot.get_link_Jacobian(self.link)[0:3]

        self.err = (self.getDesired() - robot.getLinkTransform(self.link)
                    [0:3, 3].reshape(3, 1)).reshape(3, 1)


class Orientation3D(Task):
    '''
        Subclass of Task, representing the 3D orientation (yaw) task.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired yaw angle
        link (integer): link index passed to the robot model
    '''
    def __init__(self, name, desired,link):
        super().__init__(name, desired, activation=True)
        self.err = np.zeros((1)) # Initialize with proper dimensions
        self.link = link
        self.activation_function = True
        self.limit_activation = 1


    def update(self, robot):
        '''
            Method updating the task Jacobian and error from the robot state.
        '''
        self.J = robot.get_link_Jacobian(self.link)[-1].reshape(1, 4)


        angle = np.arctan2(robot.getLinkTransform(self.link)[1,0], robot.getLinkTransform(self.link)[0,0])
        self.err = self.sigma_d - np.array([[angle]]) # Update task error

class Configuration3D(Task):
    '''
        Subclass of Task, representing the 3D configuration task [x, y, z, yaw].

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired configuration (4 x 1)
        link (integer): link index passed to the robot model
    '''
    def __init__(self, name, desired, link):
        super().__init__(name, desired, activation=True)
        self.link = link
        self.activation_function = True


        #self.J = # Initialize with proper dimensions
        #self.err = # Initialize with proper dimensions

    def update(self, robot):
        '''
            Method updating the task Jacobian and error from the robot state.
        '''
        self.J      = np.concatenate([robot.get_link_Jacobian(self.link)[0:3], robot.get_link_Jacobian(self.link)[-1].reshape(1, robot.getDOF())])
        sigma       = np.zeros((4,1))

        sigma[0:3] = robot.getLinkTransform(self.link)[0:3, 3].reshape(3, 1)

        sigma[3]    = np.arctan2(robot.getLinkTransform(self.link)[1,0], robot.getLinkTransform(self.link)[0,0])
        self.err    = (self.sigma_d - sigma).reshape(4,1)


class JointPosition(Task):
    '''
        Subclass of Task, representing the joint position task.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired joint position
        link (integer): 0-based index of the joint
    '''
    def __init__(self, name, desired, link):
        super().__init__(name, desired, activation=True)
        # self.J = # Initialize with proper dimensions
        # self.err = # Initialize with proper dimensions
        self.link = link

    def update(self, robot):
        '''
            Method updating the task Jacobian and error from the robot state.
        '''
        self.J = np.zeros((1, robot.getDOF()))
        self.J[0, self.link] = 1
        self.err = self.getDesired() - robot.getJointPos(self.link)


class JointLimitTask(Task):
    '''
        Subclass of Task, representing a joint-limit (set-based) task.

        The activation becomes -1 / +1 when the monitored angle gets within
        `alpha` of the upper / lower limit and returns to 0 once it is `delta`
        away from that limit.

        Arguments:
        name (string): title of the task
        threshold (list of double): [alpha, delta] activation / deactivation margins
        Q (list of double): [q_min, q_max] joint limits
        link (integer): link index passed to the robot model
    '''
    def __init__(self, name, threshold, Q, link):
        super().__init__(name, threshold)
        self.link = link
        self.alpha = threshold[0]
        self.delta = threshold[1]
        self.qi_min = Q[0]
        self.qi_max = Q[1]
        self.limit_activation = -1

    def update(self, robot):
        '''
            Method updating the task Jacobian and activation state.

            TODO: the monitored angle is the link yaw computed from the link
            transformation, not the joint position stored in self.joint_position.
        '''
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


