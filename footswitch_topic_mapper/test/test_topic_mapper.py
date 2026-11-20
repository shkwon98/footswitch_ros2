"""Exercise the installed adapter with a change-only pedal and its actual QoS."""
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from footswitch_msgs.msg import FootswitchState
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushROSNamespace
from launch_ros.substitutions import FindPackageShare
from rclpy.duration import Duration
from rclpy.qos import LivelinessPolicy, QoSProfile
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    path = PathJoinSubstitution([
        FindPackageShare('footswitch_topic_mapper'), 'launch', 'mapper.launch.py',
    ])
    return launch.LaunchDescription([
        GroupAction(actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path),
                launch_arguments={
                    'launch_driver': 'false',
                    'left_binding': 'press:true:/fixture/auxiliary',
                    'middle_binding': 'release:false:/fixture/enabled',
                    'right_binding': 'release:true:/fixture/enabled',
                }.items(),
            ),
        ]),
        GroupAction(actions=[
            PushROSNamespace('unbound'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path),
                launch_arguments={'launch_driver': 'false'}.items(),
            ),
        ]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestFootswitch(unittest.TestCase):
    def test_pedal_edges_and_loss_of_polling(self):
        rclpy.init()
        node = rclpy.create_node('pedal_fixture')
        try:
            qos = QoSProfile(depth=10, liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
                             liveliness_lease_duration=Duration(seconds=0.3))
            publisher = node.create_publisher(FootswitchState, '/footswitch/_state', qos)
            samples = []
            auxiliary = []
            node.create_subscription(Bool, '/fixture/enabled',
                                     lambda msg: samples.append(msg.data), 10)
            node.create_subscription(Bool, '/fixture/auxiliary',
                                     lambda msg: auxiliary.append(msg.data), 10)

            def sample(duration, poll=True):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    if poll:
                        publisher.assert_liveliness()
                    rclpy.spin_once(node, timeout_sec=0.02)

            def state(middle, right, left=False):
                message = FootswitchState()
                message.header.stamp = node.get_clock().now().to_msg()
                message.state = [left, middle, right]
                publisher.publish(message)
                sample(0.12)

            deadline = time.monotonic() + 5.0
            while (not samples or not auxiliary or not publisher.get_subscription_count()) and time.monotonic() < deadline:
                sample(0.1)
            self.assertTrue(samples)
            self.assertTrue(auxiliary)
            unbound_node = ('pedal_topic_mapper', '/unbound/footswitch')
            while (unbound_node not in node.get_node_names_and_namespaces()
                   and time.monotonic() < deadline):
                sample(0.1)
            self.assertIn(unbound_node, node.get_node_names_and_namespaces())
            self.assertFalse(any(
                'std_msgs/msg/Bool' in types
                for _, types in node.get_publisher_names_and_types_by_node(*unbound_node)
            ))
            self.assertFalse(any(samples))
            self.assertFalse(any(auxiliary))
            state(False, False)
            state(False, False, left=True)
            self.assertTrue(auxiliary[-1])
            self.assertFalse(samples[-1])
            state(False, False)
            self.assertTrue(auxiliary[-1])
            state(False, True)
            state(False, False)
            self.assertTrue(samples[-1])
            state(True, True)
            state(False, False)
            self.assertFalse(samples[-1])
            self.assertTrue(auxiliary[-1])
            state(True, True)
            state(True, False)
            self.assertFalse(samples[-1])
            state(False, False)
            state(False, True)
            state(False, False)
            self.assertTrue(samples[-1])
            # The driver process can survive a HID read failure and stop asserting liveliness.
            sample(1.2, poll=False)
            self.assertFalse(samples[-1])
            self.assertFalse(auxiliary[-1])
            samples.clear()
            sample(0.6)
            self.assertTrue(samples)
            self.assertFalse(any(samples))
            state(False, True)
            state(False, False)
            self.assertTrue(samples[-1])
        finally:
            node.destroy_node()
            rclpy.shutdown()
