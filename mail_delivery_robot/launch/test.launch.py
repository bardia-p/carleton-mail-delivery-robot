from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    init_pos = DeclareLaunchArgument("init_pos", default_value=TextSubstitution(text="0.2:0.1"))

    collision_freq = DeclareLaunchArgument("collision_freq", default_value="0")
    
    path = DeclareLaunchArgument("path", default_value="")

    wall_diff = DeclareLaunchArgument("wall_diff", default_value="0")

    delivery = DeclareLaunchArgument("delivery", default_value="UC:Nicol")
    
    duration = DeclareLaunchArgument("duration", default_value="45")
    
    return LaunchDescription([
        init_pos,
        collision_freq,
        path,
        wall_diff,
        delivery,
        duration,
        Node(package='mail_delivery_robot',
            namespace='control',
            executable='action_translator',
            name='action_translator',
            output='log',
            remappings=[('/control/cmd_vel', '/cmd_vel')]
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
            parameters=[{"init_pos": LaunchConfiguration('init_pos')},
                        {"collision_freq": LaunchConfiguration('collision_freq')},
                        {"path": LaunchConfiguration('path')},
                        {"wall_diff": LaunchConfiguration('wall_diff')},
                        {"delivery": LaunchConfiguration('delivery')},
                        {"duration": LaunchConfiguration('duration')}],
            remappings=[('/stubs/perceptions', '/control/perceptions'),
                        ('/stubs/bumpEvent', '/control/bumpEvent'),
                        ('/stubs/beacons', '/navigation/beacons'),
                        ('/stubs/cmd_vel', '/cmd_vel')]
            ),
        ])
