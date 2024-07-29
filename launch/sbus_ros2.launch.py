from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare('sbus_serial'), 'config', 'sbus_config.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Serial port'),
        DeclareLaunchArgument('refresh_rate_hz', default_value='5', description='Refresh rate in Hz'),
        DeclareLaunchArgument('rxMinValue', default_value='172', description='RX minimum value'),
        DeclareLaunchArgument('rxMaxValue', default_value='1811', description='RX maximum value'),
        DeclareLaunchArgument('outMinValue', default_value='0', description='Output minimum value'),
        DeclareLaunchArgument('outMaxValue', default_value='1000', description='Output maximum value'),
        DeclareLaunchArgument('silentOnFailsafe', default_value='true', description='Silent on failsafe'),
        DeclareLaunchArgument('enableChannelNum', default_value='-1', description='Enable channel number'),
        DeclareLaunchArgument('enableChannelProportionalMin', default_value='0.0', description='Enable channel proportional minimum value'),
        DeclareLaunchArgument('enableChannelProportionalMax', default_value='0.0', description='Enable channel proportional maximum value'),
        DeclareLaunchArgument('params_file', default_value=default_params_file, description='Path to the YAML file with parameters'),


        Node(
            package='sbus_serial',
            executable='sbus_serial_node',
            name='sbus_node',
            parameters=[
                LaunchConfiguration('params_file')
            ]
        ),

        Node(
            package='sbus_serial',
            executable='sbus_cmd_vel_node',
            name='sbus_cmd_vel_node',
            parameters=[
                LaunchConfiguration('params_file')
            ],
            remappings=[
                ('/output/sbus/cmd_vel', '/o3/cmd_vel')
            ]   
        )
    ])
