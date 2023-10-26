import time
import math
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import sys
from bluepy.btle import Scanner, DefaultDelegate
import csv

# Class instantiation which enables handleNotification and handleDiscovery debugging logs
class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)


DEBUG = False
class BeaconSensor(Node):
    def __init__(self):
        super().__init__('beacon_sensor')
        self.beacons = {"e2:77:fc:f9:04:93": 1, "ea:2f:93:a6:98:20":2, "fc:e2:2e:62:9b:3d":3, "e4:87:91:3d:1e:d7": 4, "ee:16:86:9a:c2:a8": 5, "d0:6a:d2:02:42:eb": 6, "df:2b:70:a8:21:90":7, "fb:ef:5c:de:ef:e4":8}
        self.publisher_ = self.create_publisher(String, 'beacons', 10) # Create a publisher of String msgs named beacons
        timer_period = 0.5 # Seconds
        self.timer = self.create_timer(timer_period, self.checkForBeacons) # call checkForBeacons() every 0.5 seconds
        self.scanner = Scanner().withDelegate(ScanDelegate()) # Create Scanner Class
 

    def checkForBeacons(self):
        devices = self.scanner.scan(0.4) # Listen for ADV_IND packages for 0.4 seconds
        beaconData = String()

        # For each scanned device check if device address matches beacon in list
        for dev in devices:
            for beacon in self.beacons.keys():
                if(beacon  == dev.addr):
                    # Log successful device detection and signal strength
                    #self.get_logger().info("Device {} ({}), RSSI={} dB".format(dev.addr, dev.addrType, dev.rssi))
                    beaconData.data = beacon + "," + str(dev.rssi) # Configure message from beacon data
                    self.publisher_.publish(beaconData) # Publish Message
                    #if True:
                    #    f = open('captainLog.csv', "a") # log to captainLog.csv file
                    #    f.write(beaconData.data +"\n")
                    #    f.close()



def main(): # instantiate everything
    rclpy.init()
    beacon_sensor = BeaconSensor()

    rclpy.spin(beacon_sensor)
    


if __name__ == '__main__':
    main()
