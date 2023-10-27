import math
import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time
import csv
import os

class ActionTranslator(Node):
    def __init__(self):
        super().__init__('action_translator')
        self.drivePublisher = self.create_publisher(Twist, 'cmd_vel', 2)
        #unimplemented docking behaviour 
        self.undockPublisher = self.create_publisher(Empty, 'dock', 1)
        self.dockPublisher = self.create_publisher(Empty, 'undock', 1)
        
        self.subscription = self.create_subscription(String, 'actions', self.decodeAction, 10)

    # Decode and execute the action
    def decodeAction(self, data):
        action = str(data.data)
        emptyMessage = Empty

        split_action = action.split(":")

        if split_action[0] == "DOCK":
            self.dockPublisher.publish(emptyMessage)
            return
        elif split_action[0] == "UNDOCK":
            self.undockPublisher.publish(emptyMessage)
            return
        
        message = Twist()
        message.linear.x = 0.4 

        if split_action[0] == "R_TURN":
            message.angular.z = -4.0
        elif split_action[0] == "L_TURN":
            message.angular.z = 1.0
        elif split_action[0] == "WALL_FOLLOW":
            message.angular.z = float(split_action[1])*2
        else:
            pass

        # actionMessage = Twist()  # the mess
        # handle basic movement commands from actions topic
        #self.get_logger().info("angular.z: " + str(message.angular.z) + " ||| linear.x: " + str(message.linear.x))  
        self.drivePublisher.publish(message)

def main():
    rclpy.init()
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)


if __name__ == '__main__':
    main()
