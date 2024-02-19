from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from enum import Enum

from tools.csv_parser import loadConfig

class Bump_Event(Enum):
    '''
    An enum for the various bump events for the robot.
    '''
    PRESSED = "PRESSED"
    UNPRESSED = "UNPRESSED"

class BumperSensor(Node):
    '''
    The Node in charge of listening to the bumper sensor.

    @Subscribers:
    - Listens to /bump to read the current state of the bumper sensor.

    @Publishers:
    - Publishes new /bumpEvent messages to /bumpEvent.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('bumper_sensor')
        
        # Get the model of the robot.
        self.declare_parameter('robot_model', 'CREATE_2')
        self.robot_model = self.get_parameter('robot_model').get_parameter_value().string_value
        
        # Load the global config.
        self.config = loadConfig()

        # Sets the default values for the sensor.
        self.counter = 0
        self.lastState = ""

        # The publishers for the node.
        self.publisher_ = self.create_publisher(String, 'bumpEvent', 10)
        
        # The subscribers for the node.
        if self.robot_model != "CREATE_3":
            from create_msgs.msg import Bumper
            self.bumperSubscriber = self.create_subscription(Bumper, 'bumper', self.read_bump, 10)
        else:
            from create_msgs.msg import HazardDetection
            self.bumperSubscriber = self.create_subscription(HazardDetection, 'hazard_detection', self.read_bump, 10)
    
    def read_bump(self, data):
        '''
        The callback for /bumper.
        Reads the bump data and acts accordingly.

        @param data: The new bumper data received.
        '''
        bumpEvent = String()

        # Updates the bumper state.
        if self.robot_model == "CREATE_3":
            hazards = data.detections
            got_hazard = False
            if len(hazards) != 0:
                for h in hazards:
                    if h.type == 1 or h.type == 2:
                        got_hazard = True
                        break

            if got_hazard:
                bumpEvent.data = Bump_Event.PRESSED.value
            else:
                bumpEvent.data = Bump_Event.UNPRESSED.value
        else:
            # CREATE 1 OR CREATE 2
            # Checks to see if the bumper was pressed at all.
            is_pressed = False
            is_pressed = is_pressed or data.is_left_pressed
            is_pressed = is_pressed or data.is_right_pressed

            if self.robot_model == "CREATE_2":
                is_pressed = is_pressed or data.is_light_left
                is_pressed = is_pressed or data.is_light_front_left
                is_pressed = is_pressed or data.is_light_center_left
                is_pressed = is_pressed or data.is_light_center_right
                is_pressed = is_pressed or data.is_light_front_right
                is_pressed = is_pressed or data.is_light_right

            if (is_pressed):
                bumpEvent.data = Bump_Event.PRESSED.value
            else:
                bumpEvent.data = Bump_Event.UNPRESSED.value

        # Slows the publishing of the messages to ensure the detection is smooth.
        if (self.lastState != bumpEvent.data or self.counter > self.config["MAX_BUMP_COUNT"]):
            self.lastState = bumpEvent.data
            self.publisher_.publish(bumpEvent)
            self.counter = 0
        self.counter += 1

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    bumper_sensor = BumperSensor()
    rclpy.spin(bumper_sensor)


if __name__ == '__main__':
    main()
