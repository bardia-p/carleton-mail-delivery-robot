import math
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from statistics import  mean, stdev

class LidarSensor(Node):
    '''
    The Node in charge of listening to the lidar sensor.

    @Subscribers:
    - Listens to /scan for new lidar scans.

    @Publishers:
    - Publishes new updates to /perceptions.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('lidar_sensor')

        # The publishers for the node.
        self.publisher_ = self.create_publisher(String, 'perceptions' , 10)
        
        # The subscribers for the node.
        self.lidar_info_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)

    def scan_callback(self, scan):
        '''
        The callback for /scan.
        Reads the lidar scan and acts accordingly.
        '''
        calc = String()
        
        feedback, angle = self.calculate(scan)
        calc.data = str(feedback) + ":" + str(angle) 

        self.publisher_.publish(calc)

    def calculate(self, scan):
        '''
        Calculates the robot's distance with its surroundings.

        @param scan: The current lidar scan.
        ''' 
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
    '''
    Starts up the node. 
    '''
    rclpy.init()
    lidar_sensor = LidarSensor()
    rclpy.spin(lidar_sensor)
    
if __name__ == '__main__':
    main()


