import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests

class Client(Node):
    '''
    The Node in charge of communicating to the webapp.

    @Subscribers:
    - Listens to /updates for any logs to share with the web app.

    @Publishers:
    - Publishes new trip information to /trips.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('client')
        
        # The publishers for the node.
        self.tripPublisher = self.create_publisher(String, 'trips', 10)

        # The subscribers for the node.
        self.updateSubscriber = self.create_subscription(String, 'updates', self.handleUpdate, 10)

        # The timer to check for new requests with the web app.
        # TODO: REPLACE THIS WITH A CONSTANT
        self.request_timer = self.create_timer(2, self.sendRequest)

    def handleUpdate(self, data):
        '''
        The callback for /updates.

        @param data: the new update data.
        '''
        return
    
    def sendRequest(self):
        '''
        The callback for the requests timer.
        '''
        r = requests.get("https://cudelivery.azurewebsites.net/api/v1").text
        self.get_logger().info("Got a new request: " + r)

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    client = Client()
    rclpy.spin(client)


if __name__ == '__main__':
    main()
