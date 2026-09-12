"""Unit tests for the generic recorder node."""

from datetime import datetime, timezone
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter
from recorder.recorder_node import (
    next_output_uri,
    normalize_topics,
    RecorderNode,
    RecorderState,
    validate_bag_name,
)
from std_srvs.srv import SetBool


class FakeRecorder:
    """Track recorder lifecycle calls without writing a bag."""

    def __init__(self, storage_options, record_options, log_level, node_name):
        self.storage_options = storage_options
        self.record_options = record_options
        self.log_level = log_level
        self.node_name = node_name
        self.start_spin_calls = 0
        self.record_calls = 0
        self.stop_calls = 0
        self.stop_spin_calls = 0
        self.stop_error = None

    def start_spin(self):
        self.start_spin_calls += 1

    def record(self):
        self.record_calls += 1

    def stop(self):
        self.stop_calls += 1
        if self.stop_error is not None:
            raise self.stop_error

    def stop_spin(self):
        self.stop_spin_calls += 1


class RecorderFactory:
    """Create and retain fake recorder instances."""

    def __init__(self):
        self.instances = []

    def __call__(self, *args):
        recorder = FakeRecorder(*args)
        self.instances.append(recorder)
        return recorder


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Provide one ROS context for this unit-test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(tmp_path, factory, topics=None, now_factory=None):
    """Create a recorder node with deterministic test parameters."""
    overrides = [
        Parameter('bag_name', value='test_bag'),
        Parameter('topics', value=[] if topics is None else topics),
        Parameter('output_directory', value=str(tmp_path)),
    ]
    kwargs = {
        'recorder_factory': factory,
        'parameter_overrides': overrides,
    }
    if now_factory is not None:
        kwargs['now_factory'] = now_factory
    return RecorderNode(**kwargs)


def request(node, desired):
    """Invoke the SetBool callback synchronously."""
    req = SetBool.Request()
    req.data = desired
    return node._handle_set_recording(req, SetBool.Response())


def test_validation_and_topic_deduplication():
    assert validate_bag_name('hardware_run-0.7') == 'hardware_run-0.7'
    assert normalize_topics(['/joy', '/state', '/joy']) == ['/joy', '/state']
    assert normalize_topics(None) == []

    with pytest.raises(ValueError):
        validate_bag_name('../unsafe')
    with pytest.raises(ValueError):
        normalize_topics(['relative_topic'])
    with pytest.raises(Exception):
        normalize_topics(['/bad topic'])


def test_output_uri_uses_utc_and_collision_suffix(tmp_path):
    timestamp = datetime(2026, 8, 5, 1, 2, 3, 456789, tzinfo=timezone.utc)
    expected = tmp_path / 'run_20260805T010203456789Z'
    assert next_output_uri(tmp_path, 'run', timestamp) == expected

    expected.mkdir()
    assert next_output_uri(tmp_path, 'run', timestamp) == Path(f'{expected}_001')


def test_empty_topics_records_all_non_hidden_topics(tmp_path):
    factory = RecorderFactory()
    node = make_node(tmp_path, factory)
    try:
        response = request(node, True)
        options = factory.instances[0].record_options
        assert response.success
        assert options.all_topics is True
        assert options.all_services is False
        assert options.topics == []
        assert options.include_hidden_topics is False
        assert options.is_discovery_disabled is False
    finally:
        node.destroy_node()


def test_explicit_topics_and_idempotent_requests(tmp_path):
    factory = RecorderFactory()
    node = make_node(tmp_path, factory, ['/joy', '/robot/state', '/joy'])
    try:
        started = request(node, True)
        already_started = request(node, True)
        options = factory.instances[0].record_options
        assert started.success
        assert already_started.success
        assert len(factory.instances) == 1
        assert options.all_topics is False
        assert options.topics == ['/joy', '/robot/state']

        stopped = request(node, False)
        already_stopped = request(node, False)
        assert stopped.success
        assert already_stopped.success
        assert factory.instances[0].stop_calls == 1
        assert factory.instances[0].stop_spin_calls == 1
    finally:
        node.destroy_node()


def test_busy_request_is_rejected(tmp_path):
    node = make_node(tmp_path, RecorderFactory())
    try:
        with node._state_lock:
            node._state = RecorderState.STARTING
        response = request(node, True)
        assert not response.success
        assert 'busy' in response.message
        with node._state_lock:
            node._state = RecorderState.IDLE
    finally:
        node.destroy_node()


def test_start_failure_returns_to_idle(tmp_path):
    def failing_factory(*args):
        del args
        raise RuntimeError('cannot open storage')

    node = make_node(tmp_path, failing_factory)
    try:
        response = request(node, True)
        assert not response.success
        assert 'cannot open storage' in response.message
        assert not node.is_recording
        assert request(node, False).success
    finally:
        node.destroy_node()


def test_stop_failure_still_cleans_up(tmp_path):
    factory = RecorderFactory()
    node = make_node(tmp_path, factory)
    try:
        assert request(node, True).success
        factory.instances[0].stop_error = RuntimeError('flush failed')
        response = request(node, False)
        assert not response.success
        assert 'flush failed' in response.message
        assert factory.instances[0].stop_spin_calls == 1
        assert not node.is_recording
    finally:
        node.destroy_node()


def test_shutdown_finalizes_active_recording(tmp_path):
    factory = RecorderFactory()
    node = make_node(tmp_path, factory)
    assert request(node, True).success

    node.destroy_node()

    assert factory.instances[0].stop_calls == 1
    assert factory.instances[0].stop_spin_calls == 1
