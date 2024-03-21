from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from bluepy.btle import Scanner, DefaultDelegate

from tools.csv_parser import loadBeacons, loadConfig

class ScanDelegate(DefaultDelegate):
    '''
    A default delegate for the scanner class.
    This enables handleNotification and handleDiscovery debugging logs
    '''
    def __init__(self):
        DefaultDelegate.__init__(self)

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

        # Load the global config.
        self.config = loadConfig()

        # The publishers for the node.
        self.publisher_ = self.create_publisher(String, 'beacons', 10)

        # The subscribers for the node.
        self.scanner = Scanner().withDelegate(ScanDelegate()) # Create Scanner Class

        # Timer set up.
        self.timer = self.create_timer(self.config["BEACON_SCAN_TIMER"], self.checkForBeacons) # call checkForBeacons() every 0.5 seconds

        self.scan_counter = 0
        self.scan = dict()

    def initBeacons(self):
        '''
        Initializes all the beacons and their values.
        '''
        self.beacons = loadBeacons()

    def checkForBeacons(self):
        '''
        The callback for the timer.
        Performs a scan for the available Bluetooth devices.
        '''
        devices = self.scanner.scan(self.config["BEACON_SCAN_DURATION"]) # Listen for ADV_IND packages.
        beaconData = String()

        self.scan_counter += 1

        # For each scanned device check if device address matches beacon in list
        for dev in devices:
            for beacon in self.beacons.keys():
                if(beacon  == dev.addr):
                    # Log successful device detection and signal strength
                    #self.get_logger().info("Device {} ({}), RSSI={} dB".format(dev.addr, dev.addrType, dev.rssi))

                    # Publishes the observed beacon only if it is within the RSSI range.
                    if dev.rssi > self.config["BEACON_RSSI_THRESHOLD"]:
                        key = self.beacons[beacon]
                        if key in self.scan:
                            self.scan[key] = (str(dev.rssi), self.scan[key][1] + 1)
                        else:
                            self.scan[key] = (str(dev.rssi), 1)

        if self.scan_counter == self.config["BEACON_SCAN_COUNT"]:
            best_beacon = ""
            max_count = 0
            for beacon in self.scan.keys():
                if self.scan[beacon][1] > max_count:
                    max_count = self.scan[beacon][1]
                    best_beacon = beacon + "," + self.scan[beacon][0]

            if best_beacon != "":
                beaconData.data = self.beacons[beacon] + "," + str(dev.rssi) 
                self.publisher_.publish(beaconData)
            
            self.scan = dict()
            self.scan_counter = 0

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    beacon_sensor = BeaconSensor()
    rclpy.spin(beacon_sensor)
    
if __name__ == '__main__':
    main()
