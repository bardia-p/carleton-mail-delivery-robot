import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
from tools.csv_parser import loadBeacons
from tools.nav_parser import loadConnections
from navigation.map import Map
 
class Nav_Event(Enum):
    '''
    An enum for the various navigation events for the robot.
    '''
    NAV_NONE = "NAV_NONE"
    NAV_LEFT = "NAV_LEFT"
    NAV_RIGHT = "NAV_RIGHT"
    NAV_PASS = "NAV_PASS"
    NAV_DOCK = "NAV_DOCK"
    NAV_U_TURN = "NAV_U-TURN"

class Captain(Node):
    '''
    The Node in charge of determining the robot's path. 

    @Subscribers:
    - Listens to /beacons to read new beacons.

    @Publishers:
    - Publishes new navigation messages to /navigation.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('captain')
        
        # Defines the beacon orientation hashmap.

        self.beacon_connections = loadConnections()

        # Destination route for the robot.
        self.destination = "Nicol"

        # Previous beacon for the robot.
        self.prev_beacon = "UC"

        # Routing table for the captain.
        self.map = Map()

        # The publishers for the node.
        self.mapPublisher = self.create_publisher(String, 'navigation', 10)

        # The subscribers for the node.
        self.beaconSubscriber = self.create_subscription(String, 'beacons', self.readBeacon, 10)


    def readBeacon(self, data):
        '''
        The callback for /beacons.
        Decodes the given beacon and gives the appropriate route.
        The main driver of dynamic navigation.

        @param current_beacon: the beacon to analyze.
        '''
        beacon_orientation = "0"

        current_beacon,rssi = data.data.split(",")
        self.get_logger().info("CURRENT BEACON IS " + current_beacon)

        if self.prev_beacon == "":
            beacon_orientation = "1"
        elif current_beacon == self.prev_beacon:
            return
        else:
            beacon_orientation = self.beacon_connections[current_beacon][self.prev_beacon]
            if beacon_orientation == "-":
                self.get_logger().info("ROBOT HAS BEEN MOVED")
                beacon_orientation = "1"
            direction = self.map.getDirection(current_beacon + beacon_orientation, self.destination)
            navMessage = String()
            navMessage.data = direction
            self.mapPublisher.publish(navMessage)
        self.prev_beacon = current_beacon

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    captain = Captain()
    rclpy.spin(captain)


if __name__ == '__main__':
    main()
