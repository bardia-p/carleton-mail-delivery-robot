import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

class Nav_Event(Enum):
    '''
    An enum for the various navigation events for the robot.
    '''
    NAV_NONE = "NAV_NONE"
    NAV_LEFT = "NAV_LEFT"
    NAV_RIGHT = "NAV_RIGHT"
    NAV_PASS = "NAV_PASS"
    NAV_DOCK = "NAV_DOCK"

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
        
        # Maintains the route for the robot.
        self.setRoute()


        # The publishers for the node.
        self.mapPublisher = self.create_publisher(String, 'navigation', 10)

        # The subscribers for the node.
        self.beaconSubscriber = self.create_subscription(String, 'beacons', self.readBeacon, 10)

    def setRoute(self):
        '''
        Sets the route for the robot.
        '''
        self.route = [["df:2b:70:a8:21:90", Nav_Event.NAV_LEFT.value]]

    def readBeacon(self, beacon):
        '''
        The callback for /beacons.
        Decodes the given beacon and sends the appropriate route.

        @param data: the beacon to analyze.
        '''
        if len(self.route) > 0:
            beacon_id = beacon.data.split(",")[0]
            if beacon_id == self.route[0][0]:
                beacon, direction = self.route.pop(0)
                navMessage = String()
                navMessage.data = direction
                self.mapPublisher.publish(navMessage)

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    captain = Captain()
    rclpy.spin(captain)


if __name__ == '__main__':
    main()
