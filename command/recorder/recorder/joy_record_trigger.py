#!/usr/bin/env python3
"""Translate joystick button edges into generic recorder service requests."""

from threading import Lock
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class JoyRecordTrigger(Node):
    """Toggle a recorder through SetBool on joystick button rising edges."""

    def __init__(self, *, parameter_overrides=None, client_factory=None) -> None:
        super().__init__(
            'joy_record_trigger', parameter_overrides=parameter_overrides
        )
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('record_button', 0)
        self.declare_parameter(
            'recorder_service', '/recorder/set_recording'
        )
        self.declare_parameter(
            'recording_status_topic', '/recorder/recording'
        )

        self._joy_topic = self.get_parameter('joy_topic').value
        self._record_button = self.get_parameter('record_button').value
        self._recorder_service = self.get_parameter('recorder_service').value
        status_topic = self.get_parameter('recording_status_topic').value

        if (
            isinstance(self._record_button, bool)
            or not isinstance(self._record_button, int)
            or self._record_button < 0
        ):
            raise ValueError('record_button must be a non-negative integer')

        self._lock = Lock()
        self._button_pressed = False
        self._recording: Optional[bool] = None
        self._request_in_flight = False

        if client_factory is None:
            self._client = self.create_client(SetBool, self._recorder_service)
        else:
            self._client = client_factory(SetBool, self._recorder_service)

        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_subscription = self.create_subscription(
            Bool, status_topic, self._status_callback, status_qos
        )
        self._joy_subscription = self.create_subscription(
            Joy, self._joy_topic, self._joy_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Joystick recorder trigger ready; topic={self._joy_topic}, '
            f'button={self._record_button}, service={self._recorder_service}'
        )

    def _status_callback(self, message: Bool) -> None:
        with self._lock:
            self._recording = bool(message.data)

    def _joy_callback(self, message: Joy) -> None:
        if self._record_button >= len(message.buttons):
            self.get_logger().warning(
                f'Joy message has {len(message.buttons)} buttons; '
                f'configured record_button is {self._record_button}',
                throttle_duration_sec=5.0,
            )
            return

        pressed = message.buttons[self._record_button] != 0
        with self._lock:
            rising_edge = pressed and not self._button_pressed
            self._button_pressed = pressed
            if not rising_edge:
                return
            if self._request_in_flight:
                self.get_logger().warning(
                    'Ignoring recorder toggle while a request is in flight'
                )
                return
            if self._recording is None:
                self.get_logger().warning(
                    'Recorder state is not available; toggle was not sent'
                )
                return
            desired = not self._recording

            if not self._client.service_is_ready():
                self.get_logger().warning(
                    f'Recorder service is unavailable: {self._recorder_service}'
                )
                return
            self._request_in_flight = True

        request = SetBool.Request()
        request.data = desired
        future = self._client.call_async(request)
        future.add_done_callback(
            lambda completed, target=desired: self._request_done(
                completed, target
            )
        )

    def _request_done(self, future, desired: bool) -> None:
        try:
            response = future.result()
        except Exception as exc:
            with self._lock:
                self._request_in_flight = False
            self.get_logger().error(f'Recorder request failed: {exc}')
            return

        with self._lock:
            self._request_in_flight = False
            if response.success:
                self._recording = desired

        log = self.get_logger().info if response.success else self.get_logger().error
        log(response.message)


def main(args=None) -> None:
    """Run the joystick recorder trigger."""
    rclpy.init(args=args)
    node = None
    try:
        node = JoyRecordTrigger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
