from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    robot_model = DeclareLaunchArgument("robot_model",
            default_value=TextSubstitution(text="CREATE_2"),
            description="The parameter is used to declare the robot model ('CREATE_2' or 'CREATE_3'")

    init_pos = DeclareLaunchArgument("init_pos",
            default_value=TextSubstitution(text="0.2:0.1"),
            description="Specifies the robot's initial distance and angle with the wall '[distance]:[angle]'.")

    collision_freq = DeclareLaunchArgument("collision_freq",
            default_value="0",
            description="Specifies the frequency of collisions as a percentage '[0.0 to 1.0]'.")
    
    path = DeclareLaunchArgument("path",
            default_value="",
            description="Specifies the beacons the robot sees along its path '[a:b:c:..]'.")

    wall_diff = DeclareLaunchArgument("wall_diff",
            default_value="0",
            description="Specifies the wall's inconsistency as a percentage '[0.0 to 1.0]'.")

    delivery = DeclareLaunchArgument("delivery",
            default_value="",
            description="Specifies the robot's delivery (source and destination) '[src:dest]'.")
    
    duration = DeclareLaunchArgument("duration",
            default_value="45",
            description="Specifies the total time for the simulation '[value in seconds]'.")
    
    return LaunchDescription([
        robot_model,
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
            parameters=[{"robot_model": LaunchConfiguration('robot_model')}],
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
            remappings=[('/navigation/navigation', '/control/navigation'),
                        ('/navigation/update', '/communication/update')]
            ),
        Node(package='mail_delivery_robot',
            namespace='communication',
            executable='client',
            name='client',
            output='log',
            remappings=[('/communication/trips', '/navigation/trips')]
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
                        ('/stubs/trips', '/navigation/trips'),
                        ('/stubs/cmd_vel', '/cmd_vel')]
            ),
        ])
