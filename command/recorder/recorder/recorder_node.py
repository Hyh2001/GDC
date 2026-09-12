#!/usr/bin/env python3
"""Service-controlled ROS 2 bag recorder."""

from datetime import datetime, timezone
from enum import auto, Enum
import os
from pathlib import Path
import re
from threading import Lock
from typing import Callable, Iterable, Optional

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.validate_topic_name import validate_topic_name
import rosbag2_py
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


_BAG_NAME_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')


class RecorderState(Enum):
    """Internal recorder lifecycle states."""

    IDLE = auto()
    STARTING = auto()
    RECORDING = auto()
    STOPPING = auto()


def validate_bag_name(name: str) -> str:
    """Validate and return a safe bag base name."""
    if not isinstance(name, str) or not _BAG_NAME_PATTERN.fullmatch(name):
        raise ValueError(
            'bag_name must start with an alphanumeric character and contain '
            'only letters, numbers, dots, underscores, and hyphens'
        )
    return name


def normalize_topics(topics: Optional[Iterable[str]]) -> list[str]:
    """Validate absolute topic names and remove duplicates in input order."""
    if topics is None:
        return []
    if not isinstance(topics, (list, tuple)):
        raise ValueError('topics must be an array of strings')
    normalized = []
    seen = set()
    for topic in topics:
        if not isinstance(topic, str):
            raise ValueError('topics must be an array of strings')
        if not topic.startswith('/'):
            raise ValueError(f'topic must be fully qualified: {topic!r}')
        validate_topic_name(topic)
        if topic not in seen:
            normalized.append(topic)
            seen.add(topic)
    return normalized


def next_output_uri(
    output_directory: Path,
    bag_name: str,
    timestamp: datetime,
) -> Path:
    """Return a timestamped output URI that does not already exist."""
    timestamp_utc = timestamp.astimezone(timezone.utc)
    stamp = timestamp_utc.strftime('%Y%m%dT%H%M%S%fZ')
    base = output_directory / f'{bag_name}_{stamp}'
    candidate = base
    suffix = 1
    while candidate.exists():
        candidate = Path(f'{base}_{suffix:03d}')
        suffix += 1
    return candidate


class RecorderNode(Node):
    """Record ROS topics in response to an input-agnostic SetBool service."""

    def __init__(
        self,
        *,
        recorder_factory: Callable = rosbag2_py.Recorder,
        now_factory: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
        parameter_overrides=None,
    ) -> None:
        super().__init__('recorder', parameter_overrides=parameter_overrides)

        self.declare_parameter('bag_name', '')
        # Jazzy infers an empty list as BYTE_ARRAY. Dynamic typing permits both
        # the empty all-topics value and a non-empty STRING_ARRAY override.
        self.declare_parameter(
            'topics',
            [],
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('output_directory', '~/rosbags')

        self._bag_name = validate_bag_name(
            self.get_parameter('bag_name').value
        )
        topic_value = self.get_parameter('topics').value
        self._topics = normalize_topics(topic_value)
        self._output_directory = Path(
            self.get_parameter('output_directory').value
        ).expanduser().resolve()
        self._prepare_output_directory()

        self._recorder_factory = recorder_factory
        self._now_factory = now_factory
        self._recorder = None
        self._active_uri: Optional[Path] = None
        self._state = RecorderState.IDLE
        self._state_lock = Lock()

        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = self.create_publisher(
            Bool, '~/recording', status_qos
        )
        self._service = self.create_service(
            SetBool, '~/set_recording', self._handle_set_recording
        )
        self._publish_status(False)

        topic_description = (
            ', '.join(self._topics) if self._topics else 'all non-hidden topics'
        )
        self.get_logger().info(
            f'Recorder ready; output={self._output_directory}, '
            f'topics={topic_description}'
        )

    @property
    def is_recording(self) -> bool:
        """Return whether the recorder is currently recording."""
        with self._state_lock:
            return self._state is RecorderState.RECORDING

    @property
    def active_uri(self) -> Optional[Path]:
        """Return the active bag URI, if any."""
        with self._state_lock:
            return self._active_uri

    def _prepare_output_directory(self) -> None:
        try:
            self._output_directory.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise ValueError(
                f'cannot create output_directory {self._output_directory}: {exc}'
            ) from exc
        if not self._output_directory.is_dir():
            raise ValueError(
                f'output_directory is not a directory: {self._output_directory}'
            )
        if not os.access(self._output_directory, os.W_OK):
            raise ValueError(
                f'output_directory is not writable: {self._output_directory}'
            )

    def _handle_set_recording(self, request, response):
        desired = bool(request.data)
        with self._state_lock:
            if self._state in {RecorderState.STARTING, RecorderState.STOPPING}:
                response.success = False
                response.message = f'recorder is busy ({self._state.name.lower()})'
                return response
            if desired and self._state is RecorderState.RECORDING:
                response.success = True
                response.message = 'recorder is already recording'
                return response
            if not desired and self._state is RecorderState.IDLE:
                response.success = True
                response.message = 'recorder is already stopped'
                return response
            self._state = (
                RecorderState.STARTING if desired else RecorderState.STOPPING
            )

        if desired:
            return self._start_recording(response)
        return self._stop_recording(response)

    def _build_record_options(self):
        options = rosbag2_py.RecordOptions()
        options.all_topics = not self._topics
        options.all_services = False
        options.topics = list(self._topics)
        options.include_hidden_topics = False
        options.is_discovery_disabled = False
        options.disable_keyboard_controls = True
        return options

    def _start_recording(self, response):
        uri = next_output_uri(
            self._output_directory, self._bag_name, self._now_factory()
        )
        recorder = None
        try:
            storage_options = rosbag2_py.StorageOptions(
                uri=str(uri), storage_id='mcap'
            )
            record_options = self._build_record_options()
            recorder = self._recorder_factory(
                storage_options,
                record_options,
                'info',
                'managed_rosbag2_recorder',
            )
            recorder.start_spin()
            recorder.record()
        except Exception as exc:  # rosbag2_py exposes native exceptions
            self._cleanup_failed_start(recorder)
            with self._state_lock:
                self._state = RecorderState.IDLE
                self._recorder = None
                self._active_uri = None
            self._publish_status(False)
            response.success = False
            response.message = f'failed to start recording: {exc}'
            self.get_logger().error(response.message)
            return response

        with self._state_lock:
            self._recorder = recorder
            self._active_uri = uri
            self._state = RecorderState.RECORDING
        self._publish_status(True)
        response.success = True
        response.message = f'recording to {uri}'
        self.get_logger().info(response.message)
        return response

    def _cleanup_failed_start(self, recorder) -> None:
        if recorder is None:
            return
        try:
            recorder.stop()
        except Exception:
            pass
        try:
            recorder.stop_spin()
        except Exception:
            pass

    def _stop_recording(self, response):
        with self._state_lock:
            recorder = self._recorder
            uri = self._active_uri

        errors = []
        if recorder is not None:
            try:
                recorder.stop()
            except Exception as exc:  # pragma: no cover - native failure path
                errors.append(f'stop failed: {exc}')
            try:
                recorder.stop_spin()
            except Exception as exc:  # pragma: no cover - native failure path
                errors.append(f'stop_spin failed: {exc}')

        with self._state_lock:
            self._recorder = None
            self._active_uri = None
            self._state = RecorderState.IDLE
        self._publish_status(False)

        if errors:
            response.success = False
            response.message = '; '.join(errors)
            self.get_logger().error(response.message)
        else:
            response.success = True
            response.message = f'recording finalized at {uri}'
            self.get_logger().info(response.message)
        return response

    def _publish_status(self, recording: bool) -> None:
        message = Bool()
        message.data = recording
        self._status_publisher.publish(message)

    def destroy_node(self):
        """Finalize an active bag before destroying the ROS node."""
        with self._state_lock:
            should_stop = self._state is RecorderState.RECORDING
            if should_stop:
                self._state = RecorderState.STOPPING
        if should_stop:
            response = SetBool.Response()
            self._stop_recording(response)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the recorder node."""
    rclpy.init(args=args)
    node = None
    try:
        node = RecorderNode()
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
