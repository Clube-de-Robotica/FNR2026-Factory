"""
mecanum_driver.launch.py
------------------------
Launches both the mecanum_driver node (serial ↔ Arduino) and the
mecanum_driver node (cmd_vel ↔ odometry) with fully configurable parameters.

Typical usage
-------------
  ros2 launch mecanum_driver mecanum_driver.launch.py

Override any parameter on the command line, e.g.:
  ros2 launch mecanum_driver mecanum_driver.launch.py \
      serial_port:=/dev/ttyUSB0 \
      baud_rate:=115200 \
      wheel_separation_x:=0.170 \
      max_motor_speed:=12.0

Node graph
----------
  [mecanum_driver]  ←─── /motor{0-3}/cmd   ───  [mecanum_driver]
  [mecanum_driver]  ───► /motor{0-3}/encoder ──► [mecanum_driver]
  [mecanum_driver] ◄── /cmd_vel
  [mecanum_driver] ──► /odom + /tf
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ================================================================== #
    # Launch arguments                                                    #
    # ================================================================== #

    # ---- mecanum_driver parameters -------------------------------------- #
    motor_args = [
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='USB serial device for the Arduino motor controller '
                        '(e.g. /dev/ttyACM0, /dev/ttyUSB0)'),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial baud rate'),
        DeclareLaunchArgument(
            'cmd_timeout',
            default_value='1.0',
            description='Serial read timeout [s]'),
        DeclareLaunchArgument(
            'reconnect_delay',
            default_value='2.0',
            description='Seconds to wait after opening serial port for Arduino reset'),
        DeclareLaunchArgument(
            'encoder_poll_rate',
            default_value='10.0',
            description='Encoder polling frequency [Hz]'),
    ]

    # ---- mecanum_driver parameters ------------------------------------ #
    mecanum_args = [
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.0325',
            description='Wheel radius [m]  (Ø65 mm → r = 0.0325 m)'),
        DeclareLaunchArgument(
            'wheel_separation_x',
            default_value='0.160',
            description='Half track-width: distance from robot centre to left/right wheel [m]'),
        DeclareLaunchArgument(
            'wheel_separation_y',
            default_value='0.140',
            description='Half wheelbase: distance from robot centre to front/rear wheel [m]'),
        DeclareLaunchArgument(
            'max_motor_speed',
            default_value='10.0',
            description='Wheel angular speed [rad/s] that maps to PWM ±255'),
        DeclareLaunchArgument(
            'encoder_ppr',
            default_value='234.3',
            description='Encoder pulses per wheel revolution (11 pulses × gear ratio 21.3)'),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='odom',
            description='Odometry frame id'),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='base_link',
            description='Robot base frame id'),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Broadcast odom → base_link TF transform'),
    ]

    # ================================================================== #
    # Nodes                                                               #
    # ================================================================== #

    motor_driver_node = Node(
        package='mecanum_driver',
        executable='motor_driver',
        name='motor_driver',
        output='screen',
        parameters=[{
            'serial_port':       LaunchConfiguration('serial_port'),
            'baud_rate':         LaunchConfiguration('baud_rate'),
            'cmd_timeout':       LaunchConfiguration('cmd_timeout'),
            'reconnect_delay':   LaunchConfiguration('reconnect_delay'),
            'encoder_poll_rate': LaunchConfiguration('encoder_poll_rate'),
        }],
    )

    mecanum_driver_node = Node(
        package='mecanum_driver',
        executable='mecanum_driver',
        name='mecanum_driver',
        output='screen',
        parameters=[{
            'wheel_radius':       LaunchConfiguration('wheel_radius'),
            'wheel_separation_x': LaunchConfiguration('wheel_separation_x'),
            'wheel_separation_y': LaunchConfiguration('wheel_separation_y'),
            'max_motor_speed':    LaunchConfiguration('max_motor_speed'),
            'encoder_ppr':        LaunchConfiguration('encoder_ppr'),
            'odom_frame_id':      LaunchConfiguration('odom_frame_id'),
            'base_frame_id':      LaunchConfiguration('base_frame_id'),
            'publish_tf':         LaunchConfiguration('publish_tf'),
        }],
    )

    return LaunchDescription(motor_args + mecanum_args + [
        motor_driver_node,
        mecanum_driver_node,
    ])