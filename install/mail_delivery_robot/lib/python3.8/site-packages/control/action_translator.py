import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from control.state_machine import Action

from tools.csv_parser import loadConfig

class ActionTranslator(Node):
    '''
    The Node in charge of translating the actions sent by the state machine.
    
    @Subscribers:
    - Listens to /actions in case new actions were sent.

    @Publishers:
    - Publishes Twist commands to /cmds_vel to move the robot.
    - Publishes to /dock to dock the robot.
    - Publishes to /undock to undock the robot. 
    '''

    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('action_translator')

        # Load the global config.
        self.config = loadConfig()

        # The publishers for the node.
        self.drivePublisher = self.create_publisher(Twist, 'cmd_vel', 2)
        self.undockPublisher = self.create_publisher(Empty, 'dock', 1)
        self.dockPublisher = self.create_publisher(Empty, 'undock', 1)
        
        # The subscribers for the node.
        self.subscription = self.create_subscription(String, 'actions', self.decodeAction, 10)

    def decodeAction(self, data):
        '''
        The callback for /actions.
        Decodes the given action and publishes the appropriate response.

        @param data: the action data to interpret.
        '''
        action = str(data.data)
        emptyMessage = Empty

        split_action = action.split(":")

        # TODO: IMPLEMENT DOCK/UNDOCK later.
        if split_action[0] == Action.DOCK.value:
            self.dockPublisher.publish(emptyMessage)
        elif split_action[0] == Action.UNDOCK.value:
            self.undockPublisher.publish(emptyMessage)
        else:
            self.move_robot(split_action)
    
    def wall_follow(self, cur_distance, cur_angle):
        '''
        Generates a new angular velocity for the robot based on its current distance and angle with the wall.

        @param cur_distance: the robot's current distance with the wall.
        @param cur_angle: the robot's current angle with the wall.

        @return the amount the robot needs to turn by to follow the wall.
        '''
        SET_POINT = self.config["WALL_FOLLOW_SET_POINT"]
        AIM_ANGLE = self.config["WALL_FOLLOW_AIM_ANGLE"]
        ERROR = self.config["WALL_FOLLOW_SPEED"] * math.sin(AIM_ANGLE * math.pi / 180.0)

        if cur_angle > 180:
            cur_angle -= 360

        res_angle = 0
        if cur_distance > SET_POINT + ERROR:
            res_angle = -1 * AIM_ANGLE + cur_angle
        elif cur_distance < SET_POINT - ERROR:
            res_angle = AIM_ANGLE + cur_angle
        else:
            res_angle = cur_angle

        return res_angle * math.pi / 180.0
    
    def move_robot(self, move_action):
        '''
        Generates an action to move the robot with a twist command.

        @param move_action: The amount to move the robot by.
        '''
        # Generates a twist command 
        message = Twist()
        self.get_logger().info(move_action[0])
        if move_action[0] == Action.R_TURN.value:
            message.linear.x = self.config["RIGHT_TURN_LIN_SPEED"]
            message.angular.z = self.config["RIGHT_TURN_ANG_SPEED"]
        elif move_action[0] == Action.L_TURN.value:
            message.linear.x = self.config["LEFT_TURN_LIN_SPEED"]
            message.angular.z = self.config["LEFT_TURN_ANG_SPEED"]
        elif move_action[0] == Action.WALL_FOLLOW.value:
            feedback = self.wall_follow(float(move_action[1]), float(move_action[2]))

            # For minor changes avoid big arcs.
            if abs(feedback) > self.config["WALL_FOLLOW_ANGLE_CHANGE_THRESHOLD"]:
                message.linear.x = self.config["WALL_FOLLOW_SPEED"] // 2
            else:
                message.linear.x = self.config["WALL_FOLLOW_SPEED"]

            message.angular.z = feedback
        else:
            message.linear.x = self.config["WALL_FOLLOW_SPEED"]

        self.drivePublisher.publish(message)

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)

if __name__ == '__main__':
    main()
