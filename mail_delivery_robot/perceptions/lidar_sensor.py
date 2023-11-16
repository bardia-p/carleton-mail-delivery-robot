import time
import math
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from statistics import  mean, stdev

class LidarSensor(Node):
    def __init__(self):
        super().__init__('lidar_sensor')
        self.publisher_ = self.create_publisher(String, 'perceptions' , 10)
        self.lidar_info_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)

    def scan_callback(self, scan):
        self.scan = scan
        calc = String()
        
        feedback, angle = self.calculate(self.scan)
        if feedback == -1 and angle == -1: 
            calc.data = "-1:-1"
        else: 
            calc.data = str(feedback) + ":" + str(angle) 

        self.publisher_.publish(calc)

    def calculate(self, scan):     
        count = int(scan.scan_time / scan.time_increment)
        angle = 0
        min_distance = 10
        min_left_distance = 10
        min_right_distance = 10
        min_front_distance = 10

        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            #self.get_logger().info(str(degree) + "  " + str(scan.ranges[i]))
            curDir = scan.ranges[i]
            #wall_following
            if degree >= 60 and degree <= 170 and curDir < min_distance:
                min_distance = curDir
                angle = degree
            
            if (degree <= -170 or degree >= 170) and curDir < min_front_distance:
                min_front_distance = curDir
            elif degree >= 85 and degree < 95 and curDir < min_right_distance:
                min_right_distance = curDir
            elif degree > -95 and degree <= -85 and curDir < min_left_distance:
                min_left_distance = curDir
            
        if (min_left_distance > 4 or min_right_distance > 1.6) and min_front_distance >= 1:
            return -1,-1

        return min_distance, angle - 90

def main():
    rclpy.init()
    lidar_sensor = LidarSensor()

    rclpy.spin(lidar_sensor)
    
if __name__ == '__main__':
    main()


