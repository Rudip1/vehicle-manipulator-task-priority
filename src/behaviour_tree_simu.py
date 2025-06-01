#!/usr/bin/env python3

# Main tree for pick and place task
import py_trees
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import String, Float32MultiArray  
#from pick_up_objects_task.srv import Waypoint, WaypointRequest
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from time import sleep
from py_trees.composites import Sequence , Parallel , Selector
from py_trees import logging as log_tree 
from py_trees.decorators import Inverter , Retry , Timeout
from geometry_msgs.msg import PoseStamped
from hoi.msg import CustomPoseStamped 
from nav_msgs.msg import Odometry
from std_msgs.msg import String 
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
import tf
import numpy as np


# Aruco behaviour to get the aruco pose
class Move_aruco(Behaviour):
    
    def __init__(self, name):
        
        super(Move_aruco, self).__init__(name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        
        self.blackboard.register_key("tasks", access=py_trees.common.Access.READ)
        self.blackboard.register_key("tasks", access=py_trees.common.Access.WRITE)
        
        self.blackboard.register_key("index", access=py_trees.common.Access.READ)   
        self.blackboard.register_key("index", access=py_trees.common.Access.WRITE)
        
        # self.blackboard.register_key("aruco_pose", access=py_trees.common.Access.READ)
        
        self.blackboard.register_key("aruco_pose", access=py_trees.common.Access.WRITE)
                
        self.pub = rospy.Publisher('/end_effector_pose', CustomPoseStamped, queue_size=10)
        
        rospy.Subscriber('/task_feedback', String, self.callback)
        
        rospy.Subscriber('/aruco_position',PoseStamped , self.aruco_callback)
        
        self.end_effector_pose_reached = False
        
        self.publish_once = True
        self.count = 0
        self.aruco_det_cnt = 0
        self.offset = 0.29
                
        
    def setup(self):
        self.logger.debug("  %s [Pick and Place::setup()]" % self.name)
        self.blackboard.tasks = np.array([])
        self.blackboard.index = 0
        
        
    def initialise(self): 
        
        print("Move_to_aruco_initialised")
        self.location = self.blackboard.tasks
        self.end_effector_pose_reached = False
        
    
    def aruco_callback(self, msg):
        # print("aruco_callback", msg.data)
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # print("aruco_callback", x, y)
        
        self.blackboard.aruco_pose = np.array([x, y, z])        
        aruco_task = np.array([x-self.offset, 0, 0.0, 0.0, 0.0, 0.0, 2]) 
        # aruco_task = np.array([1.55, 0.00, 0.0, 0 , 0.0, 0.0, 2]), # move base 
        # print("aruco_task: ", aruco_task)   
        
        self.blackboard.tasks = np.array([aruco_task])  
        self.aruco_det_cnt += 1 
            
            
    def callback(self, msg):
        self.end_effector_pose_reached = msg.data 
        
        
    def update(self):
 
        if True and self.aruco_det_cnt > 0:
            
            # print("Location received")
            
            self.location = self.blackboard.tasks[0] # get latest task
            print("Location: ", self.location[0])  
            msg_parent = CustomPoseStamped()
            # print("Current task: ", self.location)  
            
            msg_parent.id = int(self.location[6]) # task_id
            # Create a new PoseStamped message
            msg = PoseStamped()

            # Set the fields of the PoseStamped message
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "world_ned"
            
            msg.pose.position.x = self.location[0]
            msg.pose.position.y = self.location[1]
            msg.pose.position.z = self.location[2]

            # Convert yaw angle to quaternion and set the orientation
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.location[5]) 
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            
            msg_parent.pose = msg
            
            print("msg_parent: ", msg_parent.id)
            
            # Publish the custom message
            self.pub.publish(msg_parent)
            self.publish_once = False


        if self.end_effector_pose_reached =="success":
            self.publish_once = True
            # print("move to Success ")
            # print("blackboard index_before: ", self.blackboard.index )    
            # self.blackboard.index +=  1
            # print("blackboard index_after: ", self.blackboard.index)
            # print("tasks reache ", self.location)  
            self.end_effector_pose_reached = False
            
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
                
    def terminate(self, new_status):
        self.logger.debug("  %s [Pick and Place::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
# Task behaviour class
class MoveToP1(Behaviour):
    
    def __init__(self, name):
        super(MoveToP1, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        
        self.blackboard.register_key("tasks", access=py_trees.common.Access.READ)
        self.blackboard.register_key("tasks", access=py_trees.common.Access.WRITE)
                                     
        self.blackboard.register_key("index", access=py_trees.common.Access.READ)
        self.blackboard.register_key("index", access=py_trees.common.Access.WRITE)
        
        self.blackboard.register_key("aruco_pose", access=py_trees.common.Access.READ)
        
        self.pub = rospy.Publisher('/end_effector_pose', CustomPoseStamped, queue_size=10)
        
        rospy.Subscriber('/task_feedback', String, self.callback)
        
        self.end_effector_pose_reached = False
        self.publish_once = True
        self.count = 0
        
       
    def setup(self):
        self.logger.debug("  %s [Pick and Place::setup()]" % self.name)
        # self.pub = rospy.Publisher('/end_effector_pose', CustomPoseStamped, queue_size=1)
        # rospy.Subscriber('/task_feedback', String, self.callback)        
    def initialise(self): 
        self.logger.debug("  %s [Pick and Place::initialis\e()]" % self.name)
        # self.index = self.blackboard.index
        # print("MoveToP1_initialised")
        
        # print(f'intaliaze {self.location} and {self.blackboard.index}')  
        x, y, z = self.blackboard.aruco_pose
        print("aruco_pose: ", x, y, z)
        x = x 

        z = 1.82*z
        z_offset  = z-0.20
        
        self.blackboard.tasks =  np.array([
                        
            # [0.0, 0.0, -0.25, 0.0, 0.0, 0.0,11], # approch 1        
            [x+0.08, 0.0, -0.25, 0.0, 0.0, 0.0,11], # approch 1
            [x, 0.0, -0.12, 0.0, 0.0, 0.0,11], # aprroch to pick 
            [x, 0.0, -0.4, 0.0, 0.0, 0.0,11], # move up
            [-3.14, 0.0, 0.0, 0.0, 0.0, 0.0,3], # move to the back joint
            # [0, 0.0, -0.38, 0.0, 0.0, 0.0,11],
            
            [-1.5, 2.0, 0.0, 0.0, 0.0, 0.0,3], # move to the back joint
            
            [1.55,0.0, -0.35, 0.0,0.0,0.0, 11], # move to the back joint]
            
            [0.5, 0.0, -0.35, 0.0, 0.0, 0.0,2], # move to the start 
            [1.0, 0.0, -0.38, 0.0, 0.0, 0.0,11], # move to the start
            [1.0, 0.0, -0.14, 0.0, 0.0, 0.0,11], # move down end effector
            [1.0, 0.0, -0.35, 0.0, 0.0, 0.0,11], # move up end effector
        #   [1.76, -0.09, -0.12, 0.0, 0.0, 0.0, 2], 
        #   [0.0, -0.0, -0.18, 0.0, 0.0, 0.0,0.0],
])
        
        # self.blackboard.index = 0
        self.location =self.blackboard.tasks[self.blackboard.index]
        
        self.end_effector_pose_reached = False
            
    def callback(self, msg):
        self.end_effector_pose_reached = msg.data
        
        
    def update(self):
        
        # print(f"MoveToP_update + {self.location}")
        if True or self.publish_once:
            # read location from BB and publish it 
            msg_parent = CustomPoseStamped()
            msg_parent.id = int(self.location[6]) # task_id
            # print("Cureent task: ", self.location)

            # Create a new PoseStamped message
            msg = PoseStamped()

            # Set the fields of the PoseStamped message
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "world_ned"
            
            msg.pose.position.x = self.location[0]
            msg.pose.position.y = self.location[1]
            msg.pose.position.z = self.location[2]

            # Convert yaw angle to quaternion and set the orientation
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.location[5]) 
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            
            msg_parent.pose = msg
            print("msg_parent: ", msg_parent.id)
            # Publish the custom message
            self.pub.publish(msg_parent)
        
            self.publish_once = False

        self.count = self.count + 1
        
        if self.end_effector_pose_reached =="success":
            print("move to Success ")
            print("blackboard index_before: ", self.blackboard.index )    
            self.publish_once = True
            self.blackboard.index +=  1
            print("blackboard index_after: ", self.blackboard.index)
            print("tasks reache ", self.location)  
            self.end_effector_pose_reached = False
            # print("behaviour_status: Success")    
            return py_trees.common.Status.SUCCESS
        
        elif self.end_effector_pose_reached == "Failure":
            # print("move to p1 FAILURE ")
            return py_trees.common.Status.FAILURE
            
        else:
            # print("move to p1 RUNNING ")
            return py_trees.common.Status.RUNNING
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Pick and Place::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class Pick(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Pick, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("tasks", access=py_trees.common.Access.READ)
        # self.blackboard.register_key("locations", access=py_trees.common.Access.READ)
        self.end_effector_pose_reached = False
        

    def setup(self):
        self.logger.debug("  %s [Pick::setup()]" % self.name)
        self.pub_pump = rospy.Publisher('/turtlebot/swiftpro/vacuum_gripper/pump_state', Bool, queue_size=1)
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        self.set_pump_client = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
        
    def initialise(self):
        print("PICK_initialised")
        
        self.publish_once = True
        self.logger.debug("  %s [Pick and Place2::initialise()]" % self.name)
        
    def callback(self, data):
        self.end_effector_pose_reached = data.data
        
    def update(self):
        print("Pick_update")
         # Call the service to set the pump state
        pump_state = True  # Set the desired pump state (True for on, False for off)
        response = self.set_pump_client(pump_state)
        rospy.sleep(1)
        
        if response.success:
            # rospy.loginfo('Pump state changed successfully')
            print("pick success")
            self.publish_once = False
            
            return py_trees.common.Status.SUCCESS
       
        else:
            return py_trees.common.Status.FAILURE
        
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Pick::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
        
class Place(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Place, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        # self.blackboard.register_key("location", access=py_trees.common.Access.READ)
        self.blackboard.register_key("tasks", access=py_trees.common.Access.READ)

        self.pub_pump = rospy.Publisher('/turtlebot/swiftpro/vacuum_gripper/pump_state', Bool, queue_size=10)
        
    def setup(self):
        self.logger.debug("  %s [Place::setup()]" % self.name)
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        self.set_pump_client = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
        
       
    def initialise(self):
        print("Place_initialised")
        self.publish_once = True
        # self.logger.debug("  %s [Pick and Place::initialise()]" % self.name)
    
    def callback(self, data):
        self.end_effector_pose_reached = data.data
        
    
    def update(self):
        print("Place_update")   
        # Call the service to set the pump state
        pump_state = False  # Set the desired pump state (True for on, False for off)
        response = self.set_pump_client(pump_state)
        if response.success:
            # rospy.loginfo('Pump state changed successfully')
            print("place success")
            
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Place::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")
    
    # Move_EEP = MoveToP1("Move End Effector")
    Move_Base  = Move_aruco("Move Base_aruco_pose ") 
    Move_EE = MoveToP1("Move_end-effector_Offset")
    Aprroch_EE = MoveToP1("Approch To Pick")
    Move_Up = MoveToP1("Move up Picked Up")
    
    Move_JB = MoveToP1("Rotate Joint Backward")
    move_base_place = MoveToP1("Move base Place")
    moveup = MoveToP1("Move up")    
    Move_BaseS= MoveToP1("Move EE Approach Place")
    Move_down_EE_Place = MoveToP1("Move down EE")  
    move= MoveToP1("Move to Place") 
    Move_EE_up_after_place = MoveToP1("Move EE up after place") 
    pick = Pick("pick_up")
    place   = Place("place")
    move__ = MoveToP1("Move to Place")
    
    # Create a root for the tree
    root = Sequence(name="Pick-->Transport and Place", memory=True)
    log_tree.level = log_tree.Level.DEBUG
    root.add_children(
       
        [
            # Move_EEP,
            Move_Base, # 
            Move_EE,
            Aprroch_EE,
            pick,
            Move_Up,
            
            Move_JB,
            # Moveup,
            move_base_place,
            move,
            Move_BaseS,
            move__,
            Move_down_EE_Place,
            place,
            Move_EE_up_after_place
            ])

   
    root.setup_with_descendants() 
    py_trees.display.render_dot_tree(root)

    try:
        while not rospy.is_shutdown() and  Move_EE_up_after_place.status != py_trees.common.Status.SUCCESS:
            root.tick_once()
            sleep(2) 

    except KeyboardInterrupt:
        print("Shutting down")
        rospy.signal_shutdown("KeyboardInterrupt")
        raise