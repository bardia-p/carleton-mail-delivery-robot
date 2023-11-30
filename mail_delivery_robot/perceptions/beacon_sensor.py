from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from bluepy.btle import Scanner, DefaultDelegate

class ScanDelegate(DefaultDelegate):
    '''
    A default delegate for the scanner class.
    This enables handleNotification and handleDiscovery debugging logs
    '''
    def __init__(self):
        DefaultDelegate.__init__(self)

#TODO: Replace hard coded values with a csv file that can be loaded.
class BeaconSensor(Node):
    '''
    The Node in charge of listening to beacons.

    @Subscribers:
    - Uses the Scanner to scan for Bluetooth devices.

    @Publishers:
    - Publishes to /beacons with new beacon data.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('beacon_sensor')
        
        self.initBeacons()

        # The publishers for the node.
        self.publisher_ = self.create_publisher(String, 'beacons', 10)

        # The subscribers for the node.
        self.scanner = Scanner().withDelegate(ScanDelegate()) # Create Scanner Class

        # Timer set up.
        timer_period = 0.5 # Seconds
        self.timer = self.create_timer(timer_period, self.checkForBeacons) # call checkForBeacons() every 0.5 seconds

    def initBeacons(self):
        '''
        Initializes all the beacons and their values.
        '''
        #TODO: Use a file for this.
        self.beacons = {"e2:77:fc:f9:04:93": 1, "ea:2f:93:a6:98:20":2, "fc:e2:2e:62:9b:3d":3, "e4:87:91:3d:1e:d7": 4, "ee:16:86:9a:c2:a8": 5, "d0:6a:d2:02:42:eb": 6, "df:2b:70:a8:21:90":7, "fb:ef:5c:de:ef:e4":8}

    def checkForBeacons(self):
        '''
        The callback for the timer.
        Performs a scan for the available Bluetooth devices.
        '''
        devices = self.scanner.scan(0.4) # Listen for ADV_IND packages for 0.4 seconds
        beaconData = String()

        # For each scanned device check if device address matches beacon in list
        for dev in devices:
            for beacon in self.beacons.keys():
                if(beacon  == dev.addr):
                    # Log successful device detection and signal strength
                    #self.get_logger().info("Device {} ({}), RSSI={} dB".format(dev.addr, dev.addrType, dev.rssi))

                    # Publishes the observed beacon.
                    beaconData.data = beacon + "," + str(dev.rssi) 
                    self.publisher_.publish(beaconData)

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    beacon_sensor = BeaconSensor()
    rclpy.spin(beacon_sensor)
    


if __name__ == '__main__':
    main()
