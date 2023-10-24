import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import control.state_machine as state_machine

class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')

        # configure publisher and subscribers
        self.actionPublisher = self.create_publisher(String, 'actions', 2)
        self.lidarSubscriber = self.create_subscription(String, 'perceptions', self.updateLidarSensor, 10)
        
        self.state = state_machine.FindWall(self.actionPublisher)

    def updateLidarSensor(self, data):
        if data.data == "-1":
            self.setState(self.state.lostWall())
        else:
            self.setState(self.state.gotWall(data))

    def setState(self, newState):
        if newState != self.state:
            self.state = newState
            self.get_logger().info("Changed State " + newState.printState())

def main():
    rclpy.init()
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)

if __name__ == '__main__':
    main()
