from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    robot_model = DeclareLaunchArgument("robot_model", default_value=TextSubstitution(text="CREATE_2"))
    
    return LaunchDescription([
        robot_model,
        Node(package='create_driver',
             executable='create_driver',
             name='create_driver',
             output="screen",
             parameters=[{
                 "robot_model": LaunchConfiguration('robot_model')
             }],
            remappings=[
             ('/cmd_vel', '/control/cmd_vel')]
             ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB1',
                         'serial_baudrate': 115200,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True}],
            output='screen'
            ),
        Node(package='mail_delivery_robot',
            namespace='control',
            executable='action_translator',
            name='action_translator',
            output='log'
            ),
        Node(package='mail_delivery_robot',
            namespace='control',
            executable='robot_driver',
            name='robot_driver',
            output='log'
            ),
        Node(package='mail_delivery_robot',
            namespace='navigation',
            executable='captain',
            name='captain',
            output='log',
            remappings=[('/navigation/navigation', '/control/navigation')]
            ),

        Node(package='mail_delivery_robot',
            namespace='perceptions',
            executable='lidar_sensor',
            name='lidar_sensor',
            output='log',
            remappings=[('/perceptions/perceptions', '/control/perceptions')]
            ),
        Node(package='mail_delivery_robot',
            namespace='perceptions',
            executable='beacon_sensor',
            name='beacon_sensor',
            output='log',
            remappings=[('/perceptions/beacons', '/navigation/beacons')]
            ),
        Node(package='mail_delivery_robot',
            namespace='perceptions',
            executable='bumper_sensor',
            name='bumper_sensor',
            output='log',
            remappings=[('/perceptions/bumpEvent', '/control/bumpEvent'),
                        ('/perceptions/bumper', '/bumper')]
            ),
        ])
