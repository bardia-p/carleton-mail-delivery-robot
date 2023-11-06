import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Captain(Node):
    def __init__(self):
        super().__init__('captain')
        self.mapPublisher = self.create_publisher(String, 'navigation', 10)
        self.beaconSubscriber = self.create_subscription(String, 'beacons', self.readBeacon, 10)
        self.route = [["d0:6a:d2:02:42:eb", "NAV_RIGHT"]]

    def readBeacon(self, beacon):
        if len(self.route) > 0:
            beacon_id = beacon.data.split(",")[0]
            if beacon_id == self.route[0][0]:
                beacon, direction = self.route.pop(0)
                navMessage = String()
                navMessage.data = direction
                self.mapPublisher.publish(navMessage)

def main():
    rclpy.init()
    captain = Captain()
    rclpy.spin(captain)


if __name__ == '__main__':
    main()
