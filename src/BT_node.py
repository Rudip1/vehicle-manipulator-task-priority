#!/usr/bin/env python3

import py_trees
import rospy
from std_srvs.srv import Trigger, TriggerRequest
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


class MoveToP1(Behaviour):
    def __init__(self, name):
        super(MoveToP1, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("tasks", access=py_trees.common.Access.READ)
        self.blackboard.register_key("tasks", access=py_trees.common.Access.WRITE)
# 
        self.blackboard.register_key("index", access=py_trees.common.Access.READ)
        self.blackboard.register_key("index", access=py_trees.common.Access.WRITE)
        
        self.pub = rospy.Publisher('/end_effector_pose', CustomPoseStamped, queue_size=10)
        rospy.Subscriber('/task_feedback', String, self.callback)
           
        self.end_effector_pose_reached = False
        self.publish_once = True
        self.count = 0
        
    def setup(self):
        self.logger.debug("  %s [Pick and Place::setup()]" % self.name)
        
        # print("Publish")
        # self.pub = rospy.Publisher('/end_effector_pose', CustomPoseStamped, queue_size=1)
       
        # rospy.Subscriber('/task_feedback', String, self.callback)
    
    
        # End-effector Configurations Tasks  
        # self.blackboard.tasks = np.array([
        #                 [0.6, 0.2, -0.2, 0.0, 0.0, 0.0, 4], # move base 
        #                 [0.9, 0.2, -0.2, 0.0, 0.0, np.pi/2,4], # approch 1
        #                 [1.1, 0.4, -0.2, 0.0, 0.0, np.pi/4,4], # pick
        #                 [1.4, 0.7, -0.2, 0.0, 0.0, 0.0,4], # move up
        #                 [1.6, 1.0, -0.2, 0.0, 0.0, np.pi/2,4], # move to place
                        
        #                 # [1.5, 0.0, -0.2,0.0, 0.0, 0.0,11], # aprroch to pick 
        #                 ])
        
    #    # End-effector Position Tasks
    #     self.blackboard.tasks = np.array([
    #                     [0.5, 0.6, -0.2, 0.0, 0.0, 0.0, 11], # move base 
    #                     [0.8, 0.8, -0.2, 0.0, 0.0, 0.0,11], # approch 1
    #                     [1.0, 1.1, -0.2, 0.0, 0.0, 0.0,11], # pick
    #                     [1.3, 1.7, -0.2, 0.0, 0.0, 0.0,11], # move up
    #                     [1.6, 2.0, -0.2, 0.0, 0.0, 0.0,11], # move to place
    #                     [2.0, 2.0, -0.2, 0.0, 0.0, 0.0,11], # move to place
                        
    #                     # [1.5, 0.0, -0.2,0.0, 0.0, 0.0,11], # aprroch to pick 
    #                     ])
       
    
        # # Pick and Place Tasks
        self.blackboard.tasks = np.array([
                        # [1.7, 0.0, 0.0, 0.0, 0.0, 0.0, 2], # move base 
                        # [1.8, 0.0, -0.25, 0.0, 0.0, 0.0,11], # approch 1
                        # [2.0, 0.0, -0.13, 0.0, 0.0, 0.0,11], # pick position
                        # # Pick
                        # [2.0, 0.0, -0.25, 0.0, 0.0, 0.0,11], # up after pick
                        
                        # # [np.pi, 0, 0.0, 0.0, 0.0, 0.0,3], # Base rotate
                        
                        # [1.3, 0.0, -0.15, 0.0, 0.0, 0.0,11], # base move to place
                        
                        # # [0.8, 0.0, -0.25, 0.0, 0.0, 0.0,11], # EE approch to place
                        # # [0.8, 0.0, -0.15, 0.0, 0.0, 0.0,11], # Place
                        
                        #  #place
                        # [1.3, 0.0, -0.25, 0.0, 0.0, 0.0,11], # up after place
                        

                        [1.7, 0.0, 0.0, 0.0, 0.0, 0.0, 2], # move base 
                        [1.8, 0.0, -0.25, 0.0, 0.0, 0.0,11], # approch 1
                        [2.0, 0.0, -0.13, 0.0, 0.0, 0.0,11], # pick position of box

                        # Pick
                        [1.8, 0.0, -0.3, 0.0, 0.0, 1.0,11], # up after pick
                        
                        #[np.pi, 0, 0.0, 0.0, 0.0, 0.0,3], # Base rotate
                        
                        [1.0, 0.0, -0.17, 0.0, 0.0, 0.0,11], # base move to place
                        
                        # [0.8, 0.0, -0.25, 0.0, 0.0, 0.0,11], # EE approch to place
                        # [0.8, 0.0, -0.15, 0.0, 0.0, 0.0,11], # Place
                        
                         #place
                        [1.0, 0.0, -0.15, 0.0, 0.0, 0.0,11], # up after place
                        ])    
    
        self.blackboard.index = 0
    
    
    def initialise(self): 
        # self.logger.debug("  %s [Pick and Place::initialis\e()]" % self.name)
        
        # self.index = self.blackboard.index
        print("MoveToP1_initialised")
        
        self.location =self.blackboard.tasks[self.blackboard.index]
        self.end_effector_pose_reached = False
        # print(f'intaliaze {self.location} and {self.blackboard.index}')   
         
        
    def callback(self, msg):
        
        self.end_effector_pose_reached = msg.data
    
    def update(self):
        
        # print(f"MoveToP_update + {self.location}")
        if True or self.publish_once:
            # read location from BB and publish it 
            msg_parent = CustomPoseStamped()
            msg_parent.id = int(self.location[6]) # task_id
            print("Cureent task: ", self.location)

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
    
    # print("Pick and Place Started")
    
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")
    
    # Move_Base  = MoveToP1("Move Base ") 
     # End-effector Position Tasks 
    # Move_EE = MoveToP1("End-effector position 1")
    # Aprroch_EE = MoveToP1("End-effector position 2")
    # Move_Up = MoveToP1("End-effector position 3")
    # move_p4 = MoveToP1("End-effector position 4")
    # Move_p5 = MoveToP1("End-effector position 5")
    # # move_p6 = MoveToP1("End-effector position 6")
    
    # Move_EE = MoveToP1("End-effector config 1")
    # Aprroch_EE = MoveToP1("End-effector config 2")
    # Move_Up = MoveToP1("End-effector config 3")
    # move_p4 = MoveToP1("End-effector config 4")
    # Move_p5 = MoveToP1("End-effector configj 5")
    
    Move_base = MoveToP1("Base_Move to pick")
    Aprroch_EE = MoveToP1("EE Approch to pick")
    pick_place= MoveToP1("Pick position")
    pick = Pick("pick") 
    Move_Up_after_pick = MoveToP1("Move up after pick")
    Move_base_rotate = MoveToP1("Base Rotate")  
    Move_base_to_place = MoveToP1("Base Move to place")
    EE_approch_to_place = MoveToP1("EE Approch to place")
    EE_pose_to_place = MoveToP1("EE pose to place") 
    place = Place("place")
    Move_up_after_place = MoveToP1("Move up after place")
   
    
    # Move_JB = MoveToP1("Move Joint To back")
    # Move_BaseS= MoveToP1("Move ro Start")
    # pick = Pick("pick_up")
    # place   = Place("place")
    # Move_bot = MoveToP1("Move Boto")
    # Move_uup = MoveToP1("Move uup")
    

    
    # Create a root for the tree
    root = Sequence(name="Pick and Place with Dead_Reckoning", memory=True)
    log_tree.level = log_tree.Level.DEBUG
    root.add_children(
       
        [
            # Move_Base, # 
            # Move_EE,
            # Aprroch_EE,
            # Move_Up,
            # move_p4,
            # Move_p5,
            # move_p6
            
            Move_base,
            Aprroch_EE,
            pick_place,
            pick,
            Move_Up_after_pick,
            # Move_base_rotate,
            # Move_base_to_place,
            # EE_approch_to_place,
            EE_pose_to_place,
            place,
            Move_up_after_place,
            
        ])

    root.setup_with_descendants() 
    py_trees.display.render_dot_tree(root)
    try:
        while not rospy.is_shutdown() and  Move_up_after_place.status != py_trees.common.Status.SUCCESS:
            root.tick_once()
            sleep(1) 

    except KeyboardInterrupt:
        print("Shutting down")
        rospy.signal_shutdown("KeyboardInterrupt")
        raise