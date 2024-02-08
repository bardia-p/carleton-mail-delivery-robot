import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import control.state_machine as state_machine
from navigation.captain import Nav_Event
from perceptions.bumper_sensor import Bump_Event

from tools.csv_parser import loadConfig

class RobotDriver(Node):
    '''
    The Node in charge of moving the robot based on all of its sensor data.

    @Subscribers:
    - Listens to /perceptions in case there is new lidar sensor data.
    - Listens to /navigation in case there is a new navigation event.
    - Listens to /bumpEvent in case there is a new collision event.

    @Publishers:
    - Publishes to /actions for new actions the robot must take.

    @Timers:
    - Has a 0.1 second timer to to send regular updates based on the current state of the robot.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('robot_driver')

        # Load the global config.
        self.config = loadConfig()

        # The publishers for the node.
        self.actionPublisher = self.create_publisher(String, 'actions', 2)

        # The subscribers for the node.
        self.lidarSubscriber = self.create_subscription(String, 'perceptions', self.updateLidarSensor, 10) 
        self.beaconSubscriber = self.create_subscription(String, 'navigation', self.updateNavigation, 10)
        self.beaconSubscriber = self.create_subscription(String, 'bumpEvent', self.updateCollision, 10)

        # Timer set up.
        self.timer = self.create_timer(self.config["ROBOT_DRIVER_SCAN_TIMER"], self.updateStateMachine)

        # Initialize the robot.
        self.resetRobot()
    
    def resetRobot(self):
        '''
        Initializes the robot's state and sensor data.
        '''
        self.state = state_machine.No_Dest(self.actionPublisher)
        self.wall_data = ""
        self.nav_data = Nav_Event.NAV_NONE.value
        self.bump_data = False
        self.get_logger().info("Robot Starting " + self.state.printState())
        
    def updateLidarSensor(self, data):
        '''
        The callback for /perceptions.
        Reads and updates the information sent by the lidar sensor.

        @param data: The data sent by the lidar sensor.
        '''
        #TODO: ADD PARTICLE FILTERING HERE
        lidarData = str(data.data)
        self.get_logger().info(lidarData)

        split_data = lidarData.split(":")
        
        # waiting for the lidar to callibrate
        if split_data[0] == "-1" and split_data[1] == "-1":
            return

        # check for an intersection (lost right/left wall AND the front wall)
        if (split_data[2] == "-1" or split_data[3] == "-1") and split_data[4] == "-1":
            self.wall_data = "-1:-1"
        else: # got wall
            self.wall_data = ":".join(split_data[:2])

    def updateNavigation(self, data):
        '''
        The callback for /navigation.
        Reads and updates the information sent by the captain.

        @param data: The data sent by the captain.
        '''
        navData = str(data.data)
        self.get_logger().info("Got: " + navData)
        self.nav_data = navData

    def updateCollision(self, data):
        '''
        The callback for /bumperSensor.
        Reads and updates the information sent by the bumper sensor.

        @param data: The data sent by the bumper sensor..
        '''
        bumpData = str(data.data)
        if bumpData == Bump_Event.PRESSED.value:
            self.get_logger().info("GOT COLLISION")
            self.bump_data = True
        else:
            self.bump_data = False

    def updateStateMachine(self):
        '''
        The callback for the timer.
        Sends the current state of the robot to the state machine to update the state.
        '''
        # Waiting for everything to calibrate.
        if self.wall_data == "":
            return 

        new_state = self.state.handleUpdate(self.bump_data, self.nav_data, self.wall_data)
        
        if new_state != self.state:
            self.state = new_state
            self.get_logger().info("Changed State " + new_state.printState())
        
        self.nav_data = "NAV_NONE" # Should reset the navigation data

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)

if __name__ == '__main__':
    main()
