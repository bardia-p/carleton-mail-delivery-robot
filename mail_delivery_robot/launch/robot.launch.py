from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

lidar_serial_port = ""

def launch_setup(context):
    robot_model = LaunchConfiguration('robot_model').perform(context)
    lidar_serial_port = '/dev/ttyUSB0' if robot_model == "CREATE_3"  else '/dev/ttyUSB1'

def generate_launch_description():
    robot_model = DeclareLaunchArgument("robot_model", default_value=TextSubstitution(text="CREATE_2"))

    is_create3 = IfCondition(PythonExpression(["'",LaunchConfiguration("robot_model"),"' == 'CREATE_3'"]))

    not_create3 = IfCondition(PythonExpression(["'",LaunchConfiguration("robot_model"),"' != 'CREATE_3'"]))

    return LaunchDescription([
        robot_model,
        OpaqueFunction(function=launch_setup),
        Node(package='create_driver',
             executable='create_driver',
             name='create_driver',
             output="screen",
             parameters=[{
                "robot_model": LaunchConfiguration('robot_model')
             }],
             condition=not_create3
             ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': lidar_serial_port,
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
            parameters=[{"robot_model": LaunchConfiguration('robot_model')}],
            remappings=[('/perceptions/bumpEvent', '/control/bumpEvent'),
                        ('/perceptions/bumper', '/bumper')]
            ),
        ])
