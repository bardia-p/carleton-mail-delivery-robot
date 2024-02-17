import os
import math
import random
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from tools.csv_parser import loadConfig

LIDAR_TIMER = 0.2
BEACON_TIMER = 15
MIN_WALL_DISTANCE = 0.1
ANGLE_ERROR = 0.6
COLLISION_PERCENTAGE = 0.3
INTERSECTION_COUNT = 1
TRIP_TIMER = 0.5

class StubSensor(Node):
    '''
    The Node in charge of simulating all the sensor data.

    @Publishers:
    - Publishes new lidar updates to /perceptions.
    - Publishes new beacon updates to /beacons.
    - Publishes new bumper updates to /bumper.

    @Subscribers:
    - Subscribes to /cmd_vel to get the robot commands.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('stub_sensor')
        
        # Load the test settings.
        self.declare_parameter('init_pos', '0.2:0.1')
        self.wall_distance, self.wall_angle = self.get_parameter('init_pos').value.split(":")

        self.declare_parameter('collision_freq', '0')
        self.collision_freq = float(self.get_parameter('collision_freq').value)

        self.declare_parameter('path', '')
        self.path = self.get_parameter('path').value.split(":")

        if self.path == ['']:
            self.path = []

        self.declare_parameter('wall_diff', '0')
        self.wall_diff = float(self.get_parameter('wall_diff').value)

        self.declare_parameter('delivery', 'UC:Nicol')
        self.source, self.destination = self.get_parameter('delivery').value.split(":")

        self.declare_parameter('duration', '45')
        self.duration = float(self.get_parameter('duration').value)

        self.get_logger().info("Init Pos (distance, angle): (" + self.wall_distance + "," + self.wall_angle + ") Collision Freq: " + str(self.collision_freq) + ", Path: " + str(self.path) + " , Wall Difficulty: " + str(self.wall_diff) + ", Delivery(src, dest): (" + self.source + "," + self.destination + "), Duration: " + str(self.duration))

        # Load the global config.
        self.config = loadConfig()

        # The publishers for the node.
        self.lidar_publisher = self.create_publisher(String, 'perceptions' , 10)
        self.beacon_publisher = self.create_publisher(String, 'beacons' , 10)
        self.bumper_publisher = self.create_publisher(String, 'bumpEvent' , 10)
        self.trip_publisher = self.create_publisher(String, 'trips', 10)

        # Timer set up.
        self.lidar_timer = self.create_timer(LIDAR_TIMER, self.lidar_callback)

        if len(self.path) > 0:
            self.beacon_timer = self.create_timer(BEACON_TIMER, self.beacon_callback)
        
        if self.collision_freq > 0:
            self.bumper_timer = self.create_timer(self.collision_freq, self.bumper_callback)

        self.trip_timer = self.create_timer(TRIP_TIMER, self.init_trip)
        self.kill_timer = self.create_timer(self.duration, self.end_tests)

        # Action Translator Subscription
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.decode_action, 10)

        # Initial wall distance and angle
        self.wall_distance = float(self.wall_distance)
        self.wall_angle = float(self.wall_angle)

        # Stores the angle and distance differences received by the robot
        self.distance_diff = 0
        self.angle_diff = 0

        # Stores the number of collision clock cycles.
        self.intersection_count = 0

    def lidar_callback(self):
        '''
        The callback for the lidar timer.
        Reads the lidar scan and acts accordingly.
        '''
        calc = String()
        
        if self.intersection_count == 0:
            new_angle = self.wall_angle + self.angle_diff * LIDAR_TIMER

            diff_probability = random.random()
            self.wall_angle = new_angle % (2 * math.pi)
            self.wall_distance += (math.sin(self.angle_diff) + math.sin(self.wall_angle) * self.distance_diff) * LIDAR_TIMER

            if diff_probability < self.wall_diff:
                sign = random.random()
                if sign <= 0.5:
                    sign = -1
                else:
                    sign = 1
                self.wall_angle += ANGLE_ERROR * sign
                self.wall_distance += math.sin(ANGLE_ERROR * sign)

            if self.wall_distance <= MIN_WALL_DISTANCE:
                self.wall_distance = MIN_WALL_DISTANCE
        
            calc.data = str(self.wall_distance) + ":" + str(self.wall_angle) + ":" + str(self.wall_distance) + ":" + str(4) + ":" + str(3)
        else:
            # Simulate an intersection
            calc.data = str(self.wall_distance) + ":" + str(self.wall_angle) + ":-1:-1:-1"
            self.intersection_count -= 1

            if self.intersection_count == 0:
                self.wall_distance = 0.2
                self.wall_angle = 1.0

        self.lidar_publisher.publish(calc)
 
    def beacon_callback(self):
        '''
        The callback for the beacon timer.
        Sends a navigation event.
        '''
        if len(self.path) > 0:
            self.intersection_count = INTERSECTION_COUNT
            calc = String()
        
            calc.data = self.path.pop(0) + "," + str(self.config["BEACON_RSSI_THRESHOLD"] + 1)

            self.beacon_publisher.publish(calc)

    def bumper_callback(self):
        '''
        The callback for the bumper timer.
        Sends a bumper event.
        '''
        calc = String()
        
        should_collide = random.random()
        if should_collide <= COLLISION_PERCENTAGE:
            calc.data = "PRESSED"
        else:
            calc.data = "UNPRESSED"

        self.bumper_publisher.publish(calc)

    def decode_action(self, data):
        '''
        The callback for cmd_vel.
        
        @param: the data recently published to the robot.
        '''
        self.angle_diff = data.angular.z
        self.distance_diff = data.linear.x

    def init_trip(self):
        '''
        The callback function to initialize a new trip.
        '''
        if self.source != "" and self.destination != "":
            trip = String()
            trip.data = self.source + ":" + self.destination
            self.trip_publisher.publish(trip)
            self.source = ""
            self.destination = ""

    def end_tests(self):
        '''
        The callback to end the tests.
        '''
        nodes = ["action_translator", "robot_driver", "captain", "stub_sensor"]
        for node in nodes:
            os.system('killall ' + node)

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    stub_sensor = StubSensor()
    rclpy.spin(stub_sensor)
    
if __name__ == '__main__':
    main()

