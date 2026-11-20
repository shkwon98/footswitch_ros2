"""Launch the HID driver and map pedal edges to configured Bool topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_driver', default_value='true', choices=['true', 'false'],
            description='Launch the USB HID driver; false uses an external state publisher.',
        ),
        *[
            DeclareLaunchArgument(
                f'{side}_binding', default_value='',
                description=f'{side.capitalize()} pedal binding as event:value:/topic; empty disables.',
            )
            for side in ('left', 'middle', 'right')
        ],
        Node(package='footswitch_driver', executable='footswitch_driver_node',
             namespace='footswitch', remappings=[('state', '_state')],
             condition=IfCondition(LaunchConfiguration('launch_driver'))),
        Node(
            package='footswitch_topic_mapper', executable='pedal_topic_mapper',
            namespace='footswitch', remappings=[('state', '_state')],
            parameters=[{
                f'{side}_binding': ParameterValue(
                    LaunchConfiguration(f'{side}_binding'), value_type=str)
                for side in ('left', 'middle', 'right')
            }],
        ),
    ])
