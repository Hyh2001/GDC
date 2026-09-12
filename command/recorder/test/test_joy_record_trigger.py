"""Unit tests for the joystick recorder adapter."""

import pytest
import rclpy
from rclpy.parameter import Parameter
from recorder.joy_record_trigger import JoyRecordTrigger
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class FakeFuture:
    """Controllable future used by the fake service client."""

    def __init__(self):
        self._callbacks = []
        self._response = None
        self._error = None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        if self._error is not None:
            raise self._error
        return self._response

    def complete(self, success=True, message='ok'):
        self._response = SetBool.Response(success=success, message=message)
        for callback in self._callbacks:
            callback(self)


class FakeClient:
    """Capture asynchronous SetBool requests."""

    def __init__(self):
        self.ready = True
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(bool(request.data))
        future = FakeFuture()
        self.futures.append(future)
        return future


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Provide one ROS context for this unit-test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(client, button=0):
    """Create a joystick adapter using a fake recorder client."""
    overrides = [Parameter('record_button', value=button)]
    return JoyRecordTrigger(
        parameter_overrides=overrides,
        client_factory=lambda service_type, service_name: client,
    )


def joy(buttons):
    """Build a Joy message with the supplied button state."""
    message = Joy()
    message.buttons = buttons
    return message


def status(recording):
    """Build a recorder status message."""
    return Bool(data=recording)


def test_rising_edges_toggle_from_published_state():
    client = FakeClient()
    node = make_node(client)
    try:
        node._status_callback(status(False))
        node._joy_callback(joy([1]))
        node._joy_callback(joy([1]))
        assert client.requests == [True]

        node._joy_callback(joy([0]))
        node._joy_callback(joy([1]))
        assert client.requests == [True]

        client.futures[0].complete(success=True, message='recording')
        node._joy_callback(joy([0]))
        node._joy_callback(joy([1]))
        assert client.requests == [True, False]
    finally:
        node.destroy_node()


def test_unknown_state_and_unavailable_service_do_not_send():
    client = FakeClient()
    node = make_node(client)
    try:
        node._joy_callback(joy([1]))
        assert client.requests == []

        node._joy_callback(joy([0]))
        node._status_callback(status(False))
        client.ready = False
        node._joy_callback(joy([1]))
        assert client.requests == []
    finally:
        node.destroy_node()


def test_short_button_array_is_ignored_without_changing_edge_state():
    client = FakeClient()
    node = make_node(client, button=2)
    try:
        node._status_callback(status(False))
        node._joy_callback(joy([]))
        node._joy_callback(joy([0, 0, 1]))
        assert client.requests == [True]
    finally:
        node.destroy_node()


def test_failed_request_preserves_last_published_state():
    client = FakeClient()
    node = make_node(client)
    try:
        node._status_callback(status(False))
        node._joy_callback(joy([1]))
        client.futures[0].complete(success=False, message='busy')

        node._joy_callback(joy([0]))
        node._joy_callback(joy([1]))
        assert client.requests == [True, True]
    finally:
        node.destroy_node()
