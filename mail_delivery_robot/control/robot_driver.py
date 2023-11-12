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
        self.beaconSubscriber = self.create_subscription(String, 'navigation', self.updateNavigation, 10)
        self.beaconSubscriber = self.create_subscription(String, 'bumpEvent', self.updateCollision, 10)

        self.state = state_machine.WallFollowing(self.actionPublisher, state_machine.Direction.NONE)
        #self.state = state_machine.Docking(self.actionPublisher)
        self.get_logger().info("Robot Starting " + self.state.printState())

    def updateLidarSensor(self, data):
        lidarData = str(data.data)
        self.get_logger().info(lidarData)
        if lidarData == "-1":
            self.setState(self.state.lostWall())
        else:
            self.setState(self.state.gotWall(lidarData))

    def updateNavigation(self, data):
        navData = str(data.data)
        self.get_logger().info(navData)
        if navData == "NAV_RIGHT":
            self.setState(self.state.gotNavRight())
        elif navData == "NAV_LEFT":
            self.setState(self.state.gotNavLeft())
        elif navData == "NAV_PASS":
            self.setState(self.state.gotNavPass())
        elif navData == "NAV_DOCK":
            self.setState(self.state.gotNavDock())

    def updateCollision(self, data):
        bumpData = str(data.data)
        self.get_logger().info(bumpData)
        if bumpData == "PRESSED":
            self.setState(self.state.gotBump())
        
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
