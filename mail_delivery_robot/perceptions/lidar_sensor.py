import math
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from statistics import  stdev

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

        self.right_distances = [2.0]
        self.left_distances = [6.0]
        self.front_distances = [2.0]

    def scan_callback(self, scan):
        '''
        The callback for /scan.
        Reads the lidar scan and acts accordingly.

        @param scan: The current lidar scan.
        '''
        calc = String()
        
        feedback, angle, right, left, front  = self.calculate(scan)
        calc.data = str(feedback) + ":" + str(angle) + ":" + str(right) + ":" + str(left) + ":" + str(front)

        self.publisher_.publish(calc)

    def calculate(self, scan):
        '''
        Calculates the robot's distance with its surroundings.

        @param scan: The current lidar scan.
        ''' 
        count = int(scan.scan_time / scan.time_increment)
        angle = 0
        min_left = 10
        min_right = 10
        min_front = 10
        min_distance = 10

        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            #self.get_logger().info(str(degree) + "  " + str(scan.ranges[i]))
            curDir = scan.ranges[i]
            if curDir == math.inf:
                continue

            #wall_following
            if degree >= 30 and degree <= 150 and curDir < min_distance:
                min_distance = curDir
                angle = degree
    
            if (degree <= -175 or degree >= 175) and curDir < min_front:
                min_front = curDir
            elif degree >= 110 and degree < 115 and curDir < min_right:
                min_right = curDir
            elif degree > -115 and degree <= -110 and curDir < min_left:
                min_left = curDir
        
        self.left_distances.append(min_left)

        self.right_distances.append(min_right)

        self.front_distances.append(min_front)

        if len(self.left_distances) >  10:
            self.left_distances.pop(0)
        else:
            return -1, -1, -1, -1, -1

        if len(self.right_distances) >  10:
            self.right_distances.pop(0)
        else:
            return -1, -1, -1, -1, -1

        if len(self.front_distances) >  10:
            self.front_distances.pop(0)
        else:
            return -1, -1, -1, -1, -1

        if min_front >= 1.0 or stdev(self.front_distances) > 0.2:
            min_front = -1
       
        self.get_logger().info(str(min_right) + " " + str(stdev(self.right_distances)))
        if min_right >= 2.0 or stdev(self.right_distances) > 0.2:
            min_right = -1

        if min_left >= 6.0 or stdev(self.left_distances) > 0.2:
            min_left = -1

        return min_distance, angle - 90, min_right, min_left, min_front

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    lidar_sensor = LidarSensor()
    rclpy.spin(lidar_sensor)
    
if __name__ == '__main__':
    main()


