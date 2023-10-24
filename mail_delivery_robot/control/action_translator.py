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

        target_distance, current_distance, current_angle = action.split(":")

        self.get_logger().info('distance: ' + current_distance)
        self.get_logger().info('angle: ' + current_angle)
        self.get_logger().info('target distance: ' + target_distance)

        message = Twist()
        message.angular.z = float(target_distance) * 2
        message.linear.x = 0.2 
        
        # Get the parameters
        #(drivePublisher, dockPublisher, undockPublisher) = args
        if action == "dock":
            self.dockPublisher.publish(emptyMessage)
        elif action == "undock":
            self.undockPublisher.publish(emptyMessage)
        else:
            # actionMessage = Twist()  # the mess
            # handle basic movement commands from actions topic
            self.get_logger().info("angular.z: " + str(message.angular.z) + " ||| linear.x: " + str(message.linear.x))  
            self.drivePublisher.publish(message)

def main():
    rclpy.init()
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)


if __name__ == '__main__':
    main()
