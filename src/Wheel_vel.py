#!/usr/bin/python3

import roslib
import rospy

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class VelocityConverter:
    """
    Class that represents a robot and converts Twist messages to wheel velocities.
    """
    def __init__(self):
        """
        Constructor for VelocityConverter class.
        """
        # Robot  parameters
        self.base_dist = 0.235
        self.wheel_radius = 0.035  

        # publisher for wheel velocities
        self.wheel_pub = rospy.Publisher('/turtlebot/kobuki/commands/wheel_velocities', Float64MultiArray, queue_size=10)
        
        # subscriber for Twist 
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.twist_msg_callback)

    def twist_msg_callback(self, msg):
        """
        Callback function for Twist message subscribe.
        Converts Twist message to left and right wheel velocities, and publishes it.
        :param msg: Twist message received by subscriber
        """
        linear_v = msg.linear.x
        angular_v = msg.angular.z
        # Calculate left and right wheel velocities
        left_lin_velocity  =  linear_v - (angular_v * self.base_dist / 2.0)
        right_lin_velocity =  linear_v + (angular_v * self.base_dist / 2.0)

        left_wheel_velocity = left_lin_velocity / self.wheel_radius
        right_wheel_velocity = right_lin_velocity / self.wheel_radius

        # Publish wheel velocities
        self.wheel_pub.publish(Float64MultiArray(data = [right_wheel_velocity, left_wheel_velocity]))


if __name__ == '__main__':
    """ 
    main function that initializes node and Twist2Wheels object, and spins node.
    """
    rospy.init_node('Twist2RobotWheels')
    robot = VelocityConverter()
    rospy.spin()
