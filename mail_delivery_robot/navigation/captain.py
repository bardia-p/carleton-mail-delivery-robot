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
    NAV_START = "NAV_START"
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
    - Listens to /trip to get a new trip.
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
        self.destination = ""

        # Previous beacon for the robot.
        self.prev_beacon = ""

        # Routing table for the captain.
        self.map = Map()

        # The publishers for the node.
        self.mapPublisher = self.create_publisher(String, 'navigation', 10)
        self.updatePublisher = self.create_publisher(String, 'update', 10)

        # The subscribers for the node.
        self.beaconSubscriber = self.create_subscription(String, 'beacons', self.readBeacon, 10)
        self.tripSubscriber = self.create_subscription(String, 'trips', self.readTrip, 10)

    def readBeacon(self, data):
        '''
        The callback for /beacons.
        Decodes the given beacon and gives the appropriate route.
        The main driver of dynamic navigation.

        @param data: the beacon to analyze.
        '''
        # No trip was defined
        if self.destination == "" or self.prev_beacon == "":
            return

        beacon_orientation = "0"

        current_beacon,rssi = data.data.split(",")

        self.get_logger().info("CURRENT BEACON IS " + current_beacon)

        self.sendUpdate("IN PROGRESS: PASSED " + current_beacon)
        
        if current_beacon == self.prev_beacon:
            return
        else:
            beacon_orientation = self.beacon_connections[current_beacon][self.prev_beacon]
            if beacon_orientation == "-":
                self.sendUpdate("ERROR: ROBOT HAS BEEN MOVED")
                beacon_orientation = "1"
            direction = self.map.getDirection(current_beacon + beacon_orientation, self.destination)
            navMessage = String()
            navMessage.data = direction
            self.mapPublisher.publish(navMessage)
        self.prev_beacon = current_beacon

    def readTrip(self, data):
        '''
        The callback for trips.
        Decodes the given route.

        @param data: the trip to analyze/
        '''
        trip = data.data.split(":")
        self.prev_beacon = trip[0]
        self.destination = trip[1]
        self.sendUpdate("STARTING A NEW TRIP FROM " + self.prev_beacon + " TO " + self.destination)
        self.get_logger().info("Got a new trip from " + self.prev_beacon + " to " + self.destination)
        navMessage = String()
        navMessage.data = Nav_Event.NAV_START.value
        self.mapPublisher.publish(navMessage)

    def sendUpdate(self, data):
        '''
        Publishes an update for the user to see.

        @param data: the new update for the user.
        '''
        update_message = String()
        update_message.data = data
        self.updatePublisher.publish(update_message)

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    captain = Captain()
    rclpy.spin(captain)


if __name__ == '__main__':
    main()
