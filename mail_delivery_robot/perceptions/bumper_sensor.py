from std_msgs.msg import String
from create_msgs.msg import Bumper
import rclpy
from rclpy.node import Node
from enum import Enum

class Bump_Event(Enum):
    '''
    An enum for the various bump events for the robot.
    '''
    PRESSED = "PRESSED"
    UNPRESSED = "UNPRESSED"

#TODO: Replace hard coded values with a csv file that can be loaded.
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

        # Sets the default values for the sensor.
        self.counter = 0
        self.lastState = ""

        # The publishers for the node.
        self.publisher_ = self.create_publisher(String, 'bumpEvent', 10)
        
        # The subscribers for the node.
        self.bumperSubscriber = self.create_subscription(Bumper, 'bumper', self.readBump, 10)

    def readBump(self, data):
        '''
        The callback for /bumper.
        Reads the bump data and acts accordingly.

        @param data: The new bumper data received.
        '''
        # Checks to see if the bumper was pressed at all.
        is_pressed = False
        is_pressed = is_pressed or data.is_left_pressed
        is_pressed = is_pressed or data.is_right_pressed
        is_pressed = is_pressed or data.is_light_left
        is_pressed = is_pressed or data.is_light_front_left
        is_pressed = is_pressed or data.is_light_center_left
        is_pressed = is_pressed or data.is_light_center_right
        is_pressed = is_pressed or data.is_light_front_right
        is_pressed = is_pressed or data.is_light_right

        # Updates the bumper state.
        bumpEvent = String()
        if (is_pressed):
            bumpEvent.data = Bump_Event.PRESSED.value
        else:
            bumpEvent.data = Bump_Event.UNPRESSED.value

        # Slows the publishing of the messages to ensure the detection is smooth.
        if (self.lastState != bumpEvent.data or self.counter > 30):
            self.lastState = bumpEvent.data
            #self.get_logger().info(bumpEvent.data)
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
