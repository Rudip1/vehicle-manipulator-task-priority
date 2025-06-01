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
def jacobianLink(T, revolute, link): # Needed in Exercise 2
    '''
        Function builds a Jacobian for the end-effector of a robot,
        described by a list of kinematic transformations and a list of joint types.

        Arguments:
        T (list of Numpy array): list of transformations along the kinematic chain of the robot (from the base frame)
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint
        link(integer): index of the link for which the Jacobian is computed

        Returns:
        (Numpy array): end-effector Jacobian
    '''
    # Code almost identical to the one from lab2_robotics...
    J = np.zeros((6, len(T)-1)) # Empty Jacobian (6 x N) where N - number of joints
    O = T[-1][:3, 3]# End-effector position   

    for i in range(link):
        # a. Extract z and o.
        z = T[i][0:3, 2]
        o = T[i][0:3, 3]
        # b. Check joint type.
        if revolute[i]:
            J1_3 = np.cross(z, (O - o))
            J[:,i] = np.concatenate((J1_3, z), axis=0)
        else:
            J[:,i] = np.concatenate((z, np.zeros(3)), axis=0)
    return J

'''
    Class representing a robotic manipulator.
'''
class Manipulator:
    '''
        Constructor.

        Arguments:
        d (Numpy array): list of displacements along Z-axis
        theta (Numpy array): list of rotations around Z-axis
        a (Numpy array): list of displacements along X-axis
        alpha (Numpy array): list of rotations around X-axis
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint
    '''
    def __init__(self, d, theta, a, alpha, revolute):
        self.d = d
        self.theta = theta
        self.a = a
        self.alpha = alpha
        self.revolute = revolute
        self.dof = len(self.revolute)
        self.q = np.zeros(self.dof).reshape(-1, 1)
        self.update(0.0, 0.0)

    '''
        Method that updates the state of the robot.

        Arguments:
        dq (Numpy array): a column vector of joint velocities
        dt (double): sampling time
    '''
    def update(self, dq, dt):
        self.q += dq * dt
        for i in range(len(self.revolute)):
            if self.revolute[i]:
                self.theta[i] = self.q[i]
            else:
                self.d[i] = self.q[i]
        self.T = kinematics(self.d, self.theta, self.a, self.alpha)

    ''' 
        Method that returns the characteristic points of the robot.
    '''
    def drawing(self):
        return robotPoints2D(self.T)

    '''
        Method that returns the end-effector Jacobian.
    '''
    def getEEJacobian(self):
        return jacobian(self.T, self.revolute)[:2, :]

    '''
        Method that returns the orientation jacobian Jacobian.
    '''
    def getOrientationJacobian(self, link):
        return self.getLinkJacobian(link)[5, :] # Orientation Jacobian
    '''
        Method that returns the configuration Jacobian.
    '''
    def getConfigurationJacobian(self , link):

        J = np.block([[self.getLinkJacobian(link)[0:2, :].reshape(2,3)], 
                      [self.getOrientationJacobian(link).reshape(1,3)]])
        return J
    '''
        Method that returns the end-effector transformation.
    '''
    def getEETransform(self):
        return self.T[-1]

    '''
        Method that returns the position of a selected joint.

        Argument:
        joint (integer): index of the joint

        Returns:
        (double): position of the joint
    '''
    def getJointPos(self, joint):
        
        return self.q[joint]
    '''
        Method that returns the transformation of a selected link.

        Argument:
        link (integer): index of the link

        Returns:
        (Numpy array): transformation of the link'''
    def getLinkTransform(self, link):
     
        return self.T[link]
    '''
        Method that returns the Jacobian of a selected link.

        Argument:
        link (integer): index of the link

        Returns:
        (Numpy array): Jacobian of the link'''
    def getLinkJacobian(self, link):
        return jacobianLink(self.T, self.revolute, link)

    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
        return self.dof

'''
    Base class representing an abstract Task.
'''
class Task:
    '''
        Constructor.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired sigma (goal)
    '''
    def __init__(self, name, desired):
        self.name = name # task title
        self.sigma_d = desired # desired sigma
        self.active_flag = 1 # active task by default
    '''
        Method updating the task variables (abstract).

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''
    def isTaskActive(self):
        return True
    def getNmae(self):
        return self.name
    
    def update(self, robot):
        pass

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
    '''
        Method setting the gain matrix K.

        Arguments:
        K(Numpy array): gain matrix
    '''
    def setGian(self, K):
        self.K = K  
    '''
        Method returning the gain matrix K.
    ''' 
    def getGain(self):
        return self.K  
    ''' 
        Method setting the feedforward velocity.

        Arguments:
        value(Numpy array): value of the feedforward velocity
    '''
    def SetFeedforwardVelocity(self, value):
        self.v_ff = value
    '''
        Method returning the feedforward velocity.
    '''
    def getFeedforwardVelocity(self):
        return self.v_ff
'''
    Subclass of Task, representing the 2D position task.
'''
class Position2D(Task):
    def __init__(self, name, desired , link=5  , v_ff = np.zeros((2,1)) , K = np.eye(2) ):
        super().__init__(name, desired )
        self.J = np.zeros((2,3))
        self.err = np.zeros((2,1))  
        self.link = link
        self.v_ff = v_ff
        self.K = K
        
    def update(self, robot):
        # self.J   = robot.getEEJacobian()
        # self.err = self.sigma_d - robot.get ()[0:2,3].reshape(2,1)
        self.J = robot.getLinkJacobian(self.link)[0:2, :]    # Update task Jacobian
        desired = self.getDesired().reshape(2,1)
        expected = robot.getLinkTransform(self.link)[0:2,-1].reshape(2,1)
        self.err = desired - expected # Update task error
     
'''
    Subclass of Task, representing the 2D orientation task.
'''
class Orientation2D(Task):
    def __init__(self, name, desired , link=5 , v_ff = np.zeros((1,1)) , K = np.eye(1) ):
        super().__init__(name, desired )
        self.J = np.zeros((1,3))
        self.err = np.zeros((1,1)) # Initialize with proper dimensions
        self.link = link
        self.v_ff = v_ff
        self.K = K
        
        
    def update(self, robot):

        self.J = robot.getOrientationJacobian(self.link).reshape(1,3) # Update task Jacobian
        T = robot.getLinkTransform(self.link)
        T1 = T[1,0]
        T0 = T[0,0]
        sigma = np.arctan2(T1,T0).reshape(1,1)
        self.err = np.array(self.sigma_d - sigma ).reshape(1,1)# Update task error

'''
    Subclass of Task, representing the 2D configuration task.
'''
class Configuration2D(Task):
    def __init__(self, name, desired , link=5 , v_ff = np.zeros((3,1)) , K = np.eye(3)):
        super().__init__(name, desired)
        self.J = np.zeros((3,3))
        self.err = np.zeros((1,1)) # Initialize with proper dimensions
        self.link = link
        self.v_ff = v_ff
        self.K = K
        
    def update(self, robot):

        self.J = robot.getConfigurationJacobian(self.link).reshape(3, robot.getDOF())# Update task Jacobian
      
        T = robot.getLinkTransform(self.link)
        T1 = T[1,0]
        T0 = T[0,0]
        sigma_or= np.arctan2(T1,T0).reshape(1,1)
        sigma_pos =robot.getLinkTransform(self.link)[0:2,-1].reshape(2,1)

        sigma = np.block([[sigma_pos], [sigma_or]]).reshape(3,1)
        self.err =  self.sigma_d - sigma # Update task error
    
''' 
    Subclass of Task, representing the joint position task.
'''
class JointPosition(Task):
    def __init__(self, name, desired , link=1 ):
        super().__init__(name, desired )
        self.J =np.zeros((1,3))# Initialize with proper dimensions
        self.err =np.zeros((1,1)) # Initialize with proper dimensions
        self.active = False
        self.link = link 
    def update(self, robot):
        self.J   =  np.zeros((1,robot.getDOF())) # jacobian
        self.J[self.link-1] = 1
        self.err =  self.sigma_d - robot.getJointPos(0)

class Obstacle2D(Task):
    def __init__(self, name, obstacle_pose , obstacle_th , link =3):
        super().__init__(name , obstacle_pose )
        self.J =np.zeros((2,3))# Initialize with proper dimensions
        self.err =np.zeros((2,1)) # Initialize with proper dimensions
        self.obstacle_pose = obstacle_pose
        self.obstacle_th   = obstacle_th
        self.activation_th = obstacle_th[0]
        self.deactivation_th = obstacle_th[1]
        self.active_flag   = 0
        self.link = link 
        self.v_ff = np.zeros((2,1))
        self.K = np.eye(2)

    def update(self, robot):
        self.J = robot.getLinkJacobian(self.link)[0:2, :] # jacobian 
        end_effector_pose = robot.getEETransform()[0:2,3].reshape(2,1) # end effector pose
        self.distance = end_effector_pose  - self.obstacle_pose # distance between end effector and obstacle pose  
        dir_vector_len = np.linalg.norm(self.distance) # length of the distance vector
        self.err = self.distance / dir_vector_len # update task error 
     
        if(self.active_flag == 0 and dir_vector_len < self.obstacle_th[0]):
            self.active_flag = 1 # activate the task
        elif(self.active_flag == 1 and dir_vector_len > self.obstacle_th[1]):
            self.active_flag = 0 # deactivate the task
        
    def isTaskActive(self): 
        # check if the task is active
        if(self.active_flag == 0):
            return False
        else:
            return True
    def distance_to_obstacle(self):
        # get distance to obstacle
        self.dis_to_obs = np.linalg.norm(self.distance ) -  self.activation_th
        return self.dis_to_obs

class JointLimit(Task):
    def __init__(self, name, velo_limit, velo_thershold, link = 3):
        super().__init__(name , desired = '' )
        ''' initialize the task'''
        self.J =np.zeros((1,3))# Initialize with proper dimensions
        self.err =np.zeros((1,1)) # Initialize with proper dimensions
        self.q_min = velo_limit[0]
        self.q_max = velo_limit[1]
        self.activation_th   = velo_thershold[0]
        self.deactivation_th = velo_thershold[1]
        self.active_flag = 0
        self.link = link
        self.v_ff = np.zeros((1,1))
        self.K = np.eye(1)

    def update(self, robot):
        ''' update the task state '''
        self.J   =  np.zeros((1,robot.getDOF())) # jacobian
        self.J[0,self.link-1] = 1
        
        qi =  robot.getJointPos(self.link-1) # joint position
        if(self.active_flag == 0 and qi >= self.q_max - self.activation_th):
            self.active_flag = -1
            
        elif(self.active_flag == 0 and qi <= self.q_min + self.activation_th):
            self.active_flag = 1

        elif(self.active_flag == -1 and qi <= self.q_max - self.deactivation_th):
            self.active_flag = 0
        elif(self.active_flag == 1 and qi >= self.q_min + self.deactivation_th):
            self.active_flag = 0

        self.err = np.array([self.active_flag]).reshape(1,1) # update task error

    def isTaskActive(self): 
        ''' check if the task is active '''
        if(self.active_flag == 0):
            return False
        else:
            print("joint limit activated")
            return True
