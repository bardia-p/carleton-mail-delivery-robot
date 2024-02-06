from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    robot_model = DeclareLaunchArgument("robot_model", default_value=TextSubstitution(text="CREATE_2"))
    
    return LaunchDescription([
        robot_model,
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
            namespace='stubs',
            executable='stub_sensor',
            name='stub_sensor',
            output='log',
            remappings=[('/stubs/perceptions', '/control/perceptions'),
                        ('/stubs/bumpEvent', '/control/bumpEvent'),
                        ('/stubs/navigation', '/control/navigation')]
            ),
        ])
