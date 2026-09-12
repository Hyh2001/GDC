"""End-to-end test for joystick-controlled MCAP recording."""

from threading import Thread
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from recorder.joy_record_trigger import JoyRecordTrigger
from recorder.recorder_node import RecorderNode
import rosbag2_py
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def wait_for(predicate, timeout=5.0):
    """Wait until a predicate is true or fail at the timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    pytest.fail('timed out waiting for integration-test condition')


def publish_button_until(publisher, button_value, predicate, timeout=5.0):
    """Publish a button state until the expected state transition occurs."""
    deadline = time.monotonic() + timeout
    message = Joy()
    message.buttons = [button_value]
    while time.monotonic() < deadline:
        publisher.publish(message)
        if predicate():
            return
        time.sleep(0.05)
    pytest.fail('timed out waiting for joystick-triggered transition')


@pytest.mark.parametrize(
    'configured_topics',
    (['/recorder_integration/data'], []),
    ids=('explicit-topics', 'all-topics'),
)
def test_joystick_toggle_produces_readable_mcap(
    tmp_path, configured_topics
):
    """Start and stop through Joy messages and inspect the resulting bag."""
    rclpy.init()
    recorder = RecorderNode(parameter_overrides=[
        Parameter('bag_name', value='integration'),
        Parameter('topics', value=configured_topics),
        Parameter('output_directory', value=str(tmp_path)),
    ])
    trigger = JoyRecordTrigger(parameter_overrides=[
        Parameter('joy_topic', value='/recorder_integration/joy'),
        Parameter('record_button', value=0),
    ])
    driver = Node('recorder_integration_driver')
    joy_publisher = driver.create_publisher(
        Joy, '/recorder_integration/joy', 10
    )
    data_publisher = driver.create_publisher(
        String, '/recorder_integration/data', 10
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(recorder)
    executor.add_node(trigger)
    executor.add_node(driver)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        wait_for(lambda: trigger._recording is False)
        publish_button_until(
            joy_publisher, 1, lambda: recorder.is_recording
        )
        publish_button_until(
            joy_publisher, 0, lambda: not trigger._button_pressed
        )

        wait_for(
            lambda: driver.count_subscribers(
                '/recorder_integration/data'
            ) >= 1
        )
        for index in range(20):
            data_publisher.publish(String(data=f'message-{index}'))
            time.sleep(0.03)

        publish_button_until(
            joy_publisher, 1, lambda: not recorder.is_recording
        )
        wait_for(lambda: not trigger._request_in_flight)
    finally:
        executor.shutdown(timeout_sec=5.0)
        spin_thread.join(timeout=5.0)
        trigger.destroy_node()
        recorder.destroy_node()
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    bag_directories = [path for path in tmp_path.iterdir() if path.is_dir()]
    assert len(bag_directories) == 1
    metadata = rosbag2_py.Info().read_metadata(
        str(bag_directories[0]), 'mcap'
    )
    topic_counts = {
        entry.topic_metadata.name: entry.message_count
        for entry in metadata.topics_with_message_count
    }
    assert topic_counts['/recorder_integration/data'] > 0
