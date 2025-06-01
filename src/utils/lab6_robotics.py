import sys
import os

# Get current file's directory (which is .../hoi/src/utils)
current_dir = os.path.dirname(os.path.abspath(__file__))

# Add current_dir to sys.path if not already added
if current_dir not in sys.path:
    sys.path.append(current_dir)

# Now import
from common_func import *


import math
deg90 = np.pi / 2


class MobileManipulator:
   

    def __init__(self, d, theta, a, alpha, revolute):
        self.d = d
        self.theta = theta
        self.a = a
        self.alpha = alpha
        self.revolute = revolute

        # List of joint types extended with base joints
        self.revoluteExt = [True, False] + self.revolute

        self.r = 0  # Distance from robot centre to manipulator base
        self.dof = len(self.revoluteExt)  # Number of DOF of the system

        # Vector of joint positions (manipulator)
        self.q = np.zeros((len(self.revolute), 1))

        # Vector of base pose (position & orientation)
        self.eta = np.zeros((3, 1))

        # Initialise robot state
        self.update(np.zeros((self.dof, 1)), 0.0)

    def update(self, dQ, dt):
        """
        Method that updates the state of the robot.

        Arguments:
        - dQ (Numpy array): a column vector of quasi velocities
        - dt (double): sampling time
        """
        # Update manipulator
        self.q += dQ[2:, 0].reshape(-1, 1) * dt
        for i in range(len(self.revolute)):
            if self.revolute[i]:
                self.theta[i] = self.q[i]
            else:
                self.d[i] = self.q[i]

        # Update mobile base pose
        forward_vel = dQ[1, 0]
        angular_vel = dQ[0, 0]
        yaw = self.eta[2, 0]

        # Update mobile base pose (move forward then rotate)
        # self.forward_rotate(dQ, dt)
        
        # self.rotate_forward(dQ, dt)
        self.rotateandmove_forward(dQ, dt)
        
        
        
# 
        # Base kinematics
        x, y, yaw = self.eta.flatten()
        Tb = self.translation(x, y) @ self.rotation('z',yaw)  # Transformation of the mobile base

        ### Additional rotations performed, to align the axis:
        # Rotate Z +90 (using the theta of the first base joint)
        # Rotate X +90 (using the alpha of the first base joint)
        ## Z now aligns with the forward velocity of the base
        # Rotate X -90 (using the alpha of the second base joint)
        ## Z is now back to vertical position
        # Rotate Z -90 (using the theta of the first manipulator joint)

        # Modify the theta of the base joint, to account for an additional Z rotation
        self.theta[0] -= deg90

        # Combined system kinematics (DH parameters extended with base DOF)
        dExt = np.concatenate([np.array([0, self.r]), self.d])
        thetaExt = np.concatenate([np.array([deg90, 0]), self.theta.reshape(4,)])
        aExt = np.concatenate([np.array([0, 0]), self.a])
        alphaExt = np.concatenate([np.array([deg90, -deg90]), self.alpha])

        self.T = kinematics(dExt, thetaExt, aExt, alphaExt, Tb)
        

    def drawing(self):
        """
        Method that returns the characteristic points of the robot.
        """
        return robotPoints2D(self.T)

    def getEEJacobian(self):
        """
        Method that returns the end-effector Jacobian.
        """
        return jacobian(self.T, self.revoluteExt)
    

    def getEETransform(self):
        """
        Method that returns the end-effector transformation.
        """
        return self.T[-1]

    def getJointPos(self, joint):
        """
        Method that returns the position of a selected joint.

        Arguments:
        - joint (integer): index of the joint

        Returns:
        - (double): position of the joint
        """
        return self.q[joint - 2]
    

    def getBasePose(self):
        """
        Method that returns the base pose (position & orientation).
        """
        return self.eta

    def getDOF(self):
        """
        Method that returns the number of degrees of freedom of the manipulator.
        """
        return self.dof

    def getLinkJacobian(self, link):
        """
        Method that returns the Jacobian of a selected link.

        Arguments:
        - link (integer): index of the link

        Returns:
        - (Numpy array): Jacobian matrix
        """
        return jacobianLink(self.T, self.revoluteExt, link)

    def getLinkTransform(self, link):
        """
        Method that returns the transformation of a selected link.

        Arguments:
        - link (integer): index of the link

        Returns:
        - (Numpy array): transformation matrix
        """
        return self.T[link]

    def getOrientationJacobian(self, link):
        """
        Method that returns the orientation Jacobian of a selected link.

        Arguments:
        - link (integer): index of the link

        Returns:
        - (Numpy array): orientation Jacobian matrix
        """
        return self.getLinkJacobian(link)[3:, :]  #! To be checked!
    

    def getConfigurationJacobian(self, link):
        """
        Method that returns the configuration Jacobian of a selected link.

        Arguments:
        - link (integer): index of the link

        Returns:
        - (Numpy array): configuration Jacobian matrix
        """
        J = np.block([[self.getLinkJacobian(link)[0:3, :].reshape(3, 6)],
                      [self.getOrientationJacobian(link).reshape(3, 6)]])
        return J

    def get2dbaseTransformation(self):
        """
        Method that returns the 2D transformation matrix of the base.

        Returns:
        - (Numpy array): 2D transformation matrix
        """
        x, y, yaw = self.eta.flatten()
        return np.array([[np.cos(yaw), -np.sin(yaw), x],
                         [np.sin(yaw), np.cos(yaw), y],
                         [0, 0, 1]])

    def translation(self, translation):
        """
        Method that returns the translation matrix.

        Arguments:
        - x (float): translation along the X-axis
        - y (float): translation along the Y-axis

        Returns:
        - (Numpy array): translation matrix
        
        """
        if len(translation) != 3:
            raise ValueError("Invalid translation. Must be a 3-element tuple.")
        
        x,y,z = translation
        
        return np.array([[1, 0, 0, x],
                         [0, 1, 0, y],
                         [0, 0, 1, z],
                         [0, 0, 0, 1]])
        
        

    # def rotation_z(self, theta):
    #     """
    #     Method that returns the rotation matrix around the Z-axis.

    #     Arguments:
    #     - theta (float): rotation angle

    #     Returns:
    #     - (Numpy array): rotation matrix
    #     """
    #     return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
    #                      [np.sin(theta), np.cos(theta), 0, 0],
    #                      [0, 0, 1, 0],
    #                      [0, 0, 0, 1]])
    
    
    def rotation(self, axis, theta):
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
    
    

    def rotate_forward(self, dQ, dt):
        """
        Method that updates the mobile base pose by rotating first and then moving forward.

        Arguments:
        - dQ (Numpy array): a column vector of quasi velocities
        - dt (double): sampling time
        """
        self.eta[2, 0] += dQ[0, 0] * dt
        self.eta[0, 0] += dQ[1, 0] * np.cos(self.eta[2, 0]) * dt
        self.eta[1, 0] += dQ[1, 0] * np.sin(self.eta[2, 0]) * dt




    def rotateandmove_forward(self, dQ, dt):
        """
        Method that updates the mobile base pose by rotating and moving forward simultaneously.

        Arguments:
        - dQ (Numpy array): a column vector of quasi velocities
        - dt (double): sampling time
        """
        if math.isclose(dQ[0, 0], 0.0):
            pass
        else:
            # Calculate radius
            self.radius = dQ[1, 0] / dQ[0, 0]
            self.eta[0, 0] += -self.radius * np.sin(self.eta[2, 0]) + self.radius * np.sin(dQ[0, 0] * dt + (self.eta[2, 0]))
            self.eta[1, 0] += self.radius * np.cos(self.eta[2, 0]) - self.radius * np.cos(dQ[0, 0] * dt + (self.eta[2, 0]))
            self.eta[2, 0] += dQ[0, 0] * dt

    def forward_rotate(self, dQ, dt):
        """
        Method that updates the mobile base pose by moving forward first and then rotating.

        Arguments:
        - dQ (Numpy array): a column vector of quasi velocities
        - dt (double): sampling time
        """
        self.eta[0, 0] += dQ[1, 0] * np.cos(self.eta[2, 0]) * dt
        self.eta[1, 0] += dQ[1, 0] * np.sin(self.eta[2, 0]) * dt
        self.eta[2, 0] += dQ[0, 0] * dt
        

