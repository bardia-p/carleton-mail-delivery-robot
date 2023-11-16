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

        timer_period = 0.1 # Seconds
        self.timer = self.create_timer(timer_period, self.updateStateMachine) # call checkForBeacons() every 0.1 seconds

        self.state = state_machine.No_Dest(self.actionPublisher)
        self.get_logger().info("Robot Starting " + self.state.printState())

        self.wall_data = "-1:-1"
        self.nav_data = "NAV_NONE"
        self.bump_data = False

    def updateLidarSensor(self, data):
        lidarData = str(data.data)
        self.get_logger().info(lidarData)
        self.wall_data = lidarData

    def updateNavigation(self, data):
        navData = str(data.data)
        self.get_logger().info(navData)
        self.nav_data = navData

    def updateCollision(self, data):
        bumpData = str(data.data)
        self.get_logger().info(bumpData)
        if bumpData == "PRESSED":
            self.bump_data = True
        else:
            self.bump_data = False

    def updateStateMachine(self):
        new_state = self.state.handleUpdate(self.bump_data, self.nav_data, self.wall_data)
        self.nav_data = "NAV_NONE" # Should reset the navigation data
        if new_state != self.state:
            self.state = new_state
            self.get_logger().info("Changed State " + new_state.printState())

def main():
    rclpy.init()
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)

if __name__ == '__main__':
    main()
