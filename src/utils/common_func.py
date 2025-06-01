import numpy as np # Import Numpy
from math import cos, sin ,tan  # Import cos and sin functions from math module

def DH(d, theta, a, alpha):
    '''
        Function builds elementary Denavit-Hartenberg transformation matrices 
        and returns the transformation matrix resulting from their multiplication.

        d (double): displacement along Z-axis
        Arguments:
        [cos(theta) , -cos(alpha)*sin(theta), sin(alpha)*sin(theta) ,a*cos(theta) ]
        a (double): displacement along X-axis
        alpha (double): rotation around X-axis

        Returns:
        (Numpy array): composition of elementary DH transformations
    '''
    # 1. Build matrices representing elementary transformations (based on input parameters).
    # 2. Multiply matrices in the correct order (result in T).
    
    T = np.array([
       [cos(theta) , -cos(alpha)*sin(theta), sin(alpha)*sin(theta) ,a*cos(theta) ] , 
       [sin(theta) ,  cos(alpha)*cos(theta) , -sin(alpha)*cos(theta) , a*sin(theta) ],
       [0,sin(alpha) , cos(alpha) , d],
       [0 , 0 , 0 , 1]])
    
    return T


def kinematics(d, theta, a, alpha , Tb=np.eye(4)):
    '''
        Functions builds a list of transformation matrices, for a kinematic chain,
        descried by a given set of Denavit-Hartenberg parameters. 
        All transformations are computed from the base frame.

        Arguments:
        d (list of double): list of displacements along Z-axis
        theta (list of double): list of rotations around Z-axis
        a (list of double): list of displacements along X-axis
        alpha (list of double): list of rotations around X-axis

        Returns:
        (list of Numpy array): list of transformations along the kinematic chain (from the base frame)
    '''
    T = [Tb] # Base transformation
    # For each set of DH parameters:
    # 1. Compute the DH transformation matrix.
    # 2. Compute the resulting accumulated transformation from the base frame.
    # 3. Append the computed transformation to T.

    # [T01 , T12 , T02 [ 0, 0] 
    

    for i in range(len(d)):

        Ti = DH(d[i],theta[i],a[i],alpha[i]) # Compute the DH transformation matrix of i.
        # print("T ", i,New_T)
        T0_i = T[-1]@Ti# Compute the resulting accumulated transformation from the base frame. 
        T.append(T0_i) # Append the computed transformation to T.
        
    return T


# Inverse kinematics
def jacobian(T,EE_p, revolute):
    '''
        Function builds a Jacobian for the end-effector of a robot,
        described by a list of kinematic transformations and a list of joint types.

        Arguments:
        T (list of Numpy array): list of transformations along the kinematic chain of the robot (from the base frame)
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint

        Returns:
        (Numpy array): end-effector Jacobian
    '''
    # 1. Initialize J and O.
    # 2. For each joint of the robot
    #   a. Extract z and o.
    #   b. Check joint type.
    #   c. Modify corresponding column of J.
    
    J = np.zeros((6, len(T)-1)) # Empty Jacobian (6 x N) where N - number of joints
    
    O = EE_p.flatten()# End-effector position   

     
    for i in range(len(T)-1):
        # a. Extract z and o.
        z = T[i][0:3, 2].reshape(1,3).flatten()
        o = T[i][0:3, 3].reshape(1,3).flatten()
        
        # b. Check joint type.
        if revolute[i]:
            # Revolute joint
            # print("Revolute joint")
            J1_3 = np.cross(z, (O - o))
            # print("J1_3", J1_3)
            # print("Z",z)
            J[:,i] = np.concatenate((J1_3, z), axis=0).reshape(6).tolist()
        else:
            J[:,i] = np.concatenate((z, np.zeros(3)), axis=0).reshape(6).tolist()
    # print("Jacobian_Base:", J)
    return J

# Damped Least-Squares
def DLS(A, damping =0.1 , W = np.eye(6)):
    '''
        Function computes the damped least-squares (DLS) solution to the matrix inverse problem.

        Arguments:
        A (Numpy array): matrix to be inverted
        damping (double): damping factor

        Returns:
        (Numpy array): inversion of the input matrix
    '''
    # Implement the formula to compute the DLS of matrix A.
    W_inv = np.linalg.inv(W)
    return A.T @ np.linalg.inv(A@A.T + damping**2 * np.eye(A.shape[0]))
# Damped Least-Squares
def W_DLS(A, damping=0.1, W=None):
    '''
    Function computes the damped least-squares (DLS) solution to the matrix inverse problem.

    Arguments:
    A (Numpy array): matrix to be inverted
    damping (double): damping factor
    W (Numpy array, optional): weight matrix (default: None)

    Returns:
    (Numpy array): inversion of the input matrix
    '''
    if W is None:
        W = np.eye(A.shape[1])
    W_inv = np.linalg.inv(W)
    return W_inv @ A.T @ np.linalg.inv(A @ W_inv @ A.T + damping**2 * np.eye(A.shape[0]))

def TransposeJacob(J):
    '''
        Function computes the transpose of the Jacobian matrix.

        Arguments:
        J (Numpy array): Jacobian matrix

        Returns:
        (Numpy array): Transpose of the input matrix
    '''
    
    return J.T

def PseudoInverseJacob(J):
    '''
        Function computes the pseudo-inverse of the Jacobian matrix.

        Arguments:
        J (Numpy array): Jacobian matrix

        Returns:
        (Numpy array): Pseudo-inverse of the input matrix
    '''
    return np.linalg.pinv(J)

def robotPoints2D(T):
    '''
        Function extracts the characteristic points of a kinematic chain on a 2D plane,
        based on the list of transformations that describe it.

        Arguments:
        T (list of Numpy array): list of transformations along the kinematic chain of the robot (from the base frame)
    
        Returns:
        (Numpy array): an array of 2D points
    ''' 
    P = np.zeros((2,len(T)))
    for i in range(len(T)):
        P[:,i] = T[i][0:2,3]
    return P

def jacobianLink(T, revolute, link): # Needed in Exercise 2
  
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