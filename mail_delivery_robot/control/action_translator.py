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
        self.get_logger().info(split_action[0])
        if split_action[0] == "R_TURN":
            message.linear.x = 0.4 
            message.angular.z = -2.0
        elif split_action[0] == "L_TURN":
            message.linear.x = 0.4
            message.angular.z = 1.0
        elif split_action[0] == "WALL_FOLLOW":
            message.linear.x = 0.4 
            feedback = self.get_new_angle(float(split_action[1]), float(split_action[2]))
            message.angular.z = feedback
        else:
            message.linear.x = 0.4 

        self.drivePublisher.publish(message)

    
    def get_new_angle(self, cur_distance, cur_angle):
        SET_POINT = 0.6
        AIM_ANGLE = 60
        ERROR = 0.4 * math.sin(AIM_ANGLE * math.pi / 180.0) / 2.0
        
        if cur_angle > 180:
            cur_angle -= 360

        res_angle = 0
        if cur_distance > SET_POINT + ERROR:
            res_angle = -1 * AIM_ANGLE + cur_angle
        elif cur_distance < SET_POINT - ERROR:
            res_angle = AIM_ANGLE + cur_angle
        else:
            res_angle = cur_angle

        return res_angle * math.pi / 180.0

def main():
    rclpy.init()
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)

if __name__ == '__main__':
    main()
