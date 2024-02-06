import math
import random
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from statistics import  stdev

from tools.csv_parser import loadConfig

class StubSensor(Node):
    '''
    The Node in charge of simulating all the sensor data.

    @Publishers:
    - Publishes new lidar updates to /perceptions.
    - Publishes new beacon updates to /beacons.
    - Publishes new bumper updates to /bumper.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('stub_sensor')

        # Load the global config.
        self.config = loadConfig()

        # The publishers for the node.
        self.lidar_publisher = self.create_publisher(String, 'perceptions' , 10)
        self.beacon_publisher = self.create_publisher(String, 'navigation' , 10)
        self.bumper_publisher = self.create_publisher(String, 'bumpEvent' , 10)

        # Timer set up.
        self.lidar_timer = self.create_timer(0.2, self.lidar_callback)
        #self.beacon_timer = self.create_timer(4, self.beacon_callback)
        self.bumper_timer = self.create_timer(0.5, self.bumper_callback)

    def lidar_callback(self):
        '''
        The callback for the lidar timer.
        Reads the lidar scan and acts accordingly.
        '''
        calc = String()
        
        calc.data = str(0.4) + ":" + str(0.1) + ":" + str(0.4) + ":" + str(4) + ":" + str(3)

        self.lidar_publisher.publish(calc)
 
    def beacon_callback(self):
        '''
        The callback for the beacon timer.
        Sends a navigation event.
        '''
        calc = String()
        
        calc.data = "NAV_LEFT"

        self.beacon_publisher.publish(calc)

    def bumper_callback(self):
        '''
        The callback for the bumper timer.
        Sends a bumper event.
        '''
        calc = String()
        
        should_collide = random.randint(1,10)
        if should_collide == 1:
            calc.data = "PRESSED"
        else:
            calc.data = "UNPRESSED"

        self.bumper_publisher.publish(calc)


def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    stub_sensor = StubSensor()
    rclpy.spin(stub_sensor)
    
if __name__ == '__main__':
    main()


