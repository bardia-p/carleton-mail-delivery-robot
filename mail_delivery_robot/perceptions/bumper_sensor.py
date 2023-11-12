from std_msgs.msg import String
from create_msgs.msg import Bumper
import rclpy
from rclpy.node import Node

class BumperSensor(Node):
    def __init__(self):
        super().__init__('bumper_sensor')
        self.counter = 0
        self.lastState = ""
        # Create subscriber, msg_type = Bumper, topic = "bumper", callback = self.readBump
        self.bumperSubscriber = self.create_subscription(Bumper, 'bumper', self.readBump, 10)

        # Publisher that sends String messages named bumpEvent
        self.publisher_ = self.create_publisher(String, 'bumpEvent', 10)
        self.get_logger().info("STARTING")

    def readBump(self, data):
        self.get_logger().info("IM HERE")
        # Bumper light sensors (Create 2 only) in order from left to right
        # Value = true if an obstacle detected
        is_pressed = False
        is_pressed = is_pressed or data.is_left_pressed
        is_pressed = is_pressed or data.is_right_pressed
        is_pressed = is_pressed or data.is_light_left
        is_pressed = is_pressed or data.is_light_front_left
        is_pressed = is_pressed or data.is_light_center_left
        is_pressed = is_pressed or data.is_light_center_right
        is_pressed = is_pressed or data.is_light_front_right
        is_pressed = is_pressed or data.is_light_right

        bumpEvent = String()
        if (is_pressed):
            bumpEvent.data = "PRESSED"
        else:
            bumpEvent.data = "UNPRESSED"

        # if(data.is_light_front_right or  is_light_center_right):
        #     message.data = "bumper detects an object to the left"
        # elif(is_light_center_left or is_light_front_left):
        #     message.data = "bumper detects an object to the right"

        # Publish the perception
        if (self.lastState != bumpEvent.data or self.counter > 30):
            self.lastState = bumpEvent.data
            self.get_logger().info(bumpEvent.data)
            self.publisher_.publish(bumpEvent)
            self.counter = 0
        self.counter += 1


def main():
    rclpy.init()
    bumper_sensor = BumperSensor()

    rclpy.spin(bumper_sensor)


if __name__ == '__main__':
    main()
