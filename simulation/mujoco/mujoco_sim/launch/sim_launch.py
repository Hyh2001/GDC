from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
import tempfile
import yaml


def _is_true(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def _load_yaml_file(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML config must contain a dictionary at top level: {path}")
    return data


def _get_nested(config, *keys, default=None):
    current = config
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def _resolve_value(cli_value, config_value, default_value=None):
    if cli_value not in ("", "none"):
        return cli_value
    if config_value is not None:
        return config_value
    return default_value


def _prepare_launch(context):
    sim_config_path = LaunchConfiguration("sim_config").perform(context)
    if sim_config_path in ("", "none"):
        raise RuntimeError("`sim_config` must be provided.")
    if not os.path.isfile(sim_config_path):
        raise RuntimeError(f"Sim config file does not exist: {sim_config_path}")

    sim_config = _load_yaml_file(sim_config_path)

    build_section = sim_config.get("build", {})
    if not isinstance(build_section, dict):
        raise RuntimeError("`build` section in sim config must be a dictionary.")
    if "resources" not in build_section:
        raise RuntimeError("`build.resources` must be provided in sim config.")

    builder_python = _resolve_value(
        LaunchConfiguration("builder_python").perform(context),
        _get_nested(sim_config, "builder", "python"),
        "/opt/venv/bin/python3",
    )
    builder_module = _resolve_value(
        LaunchConfiguration("builder_module").perform(context),
        _get_nested(sim_config, "builder", "module"),
        None,
    )
    output_mjcf_path = _resolve_value(
        LaunchConfiguration("output_mjcf_path").perform(context),
        _get_nested(sim_config, "output", "mjcf_path"),
        None,
    )
    keep_generated_mjcf = _resolve_value(
        LaunchConfiguration("keep_generated_mjcf").perform(context),
        _get_nested(sim_config, "output", "keep_generated_mjcf"),
        "false",
    )
    node_class_name = _resolve_value(
        LaunchConfiguration("node_class_name").perform(context),
        _get_nested(sim_config, "simulation", "node_class_name"),
        "PendulumSimNode",
    )
    config_path = _resolve_value(
        LaunchConfiguration("config_path").perform(context),
        _get_nested(sim_config, "simulation", "config_path"),
        "none",
    )

    if not builder_python or not os.path.exists(builder_python):
        raise RuntimeError(f"Builder Python does not exist: {builder_python}")
    if builder_module in ("", "none"):
        builder_module = None
    if builder_module is None:
        raise RuntimeError("Builder config must provide `builder.module`.")
    if config_path not in ("", "none") and not os.path.isfile(config_path):
        raise RuntimeError(f"Simulation config file does not exist: {config_path}")

    if output_mjcf_path in ("", "none", None):
        temp_file = tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False)
        temp_file.close()
        output_mjcf_path = temp_file.name

    return [
        SetLaunchConfiguration("resolved_builder_python", str(builder_python)),
        SetLaunchConfiguration("resolved_builder_module", str(builder_module or "")),
        SetLaunchConfiguration("generated_mjcf_path", str(output_mjcf_path)),
        SetLaunchConfiguration("resolved_keep_generated_mjcf", str(keep_generated_mjcf)),
        SetLaunchConfiguration("resolved_node_class_name", str(node_class_name)),
        SetLaunchConfiguration("resolved_config_path", str(config_path)),
        LogInfo(msg=f"[MJCF BUILD] sim_config: {sim_config_path}"),
        LogInfo(msg=f"[MJCF BUILD] builder_python: {builder_python}"),
        LogInfo(msg=f"[MJCF BUILD] builder_module: {builder_module}"),
        LogInfo(msg=f"[MJCF BUILD] output_mjcf_path: {output_mjcf_path}"),
    ]


def _on_builder_exit(event, context, simulation_node):
    if event.returncode != 0:
        raise RuntimeError(
            f"MJCF builder failed with exit code {event.returncode}. Simulation launch aborted."
        )
    return [
        LogInfo(msg="[MJCF BUILD] Builder finished successfully. Starting simulation."),
        simulation_node,
    ]


def _delete_generated_mjcf(context):
    keep_generated_mjcf = LaunchConfiguration("resolved_keep_generated_mjcf").perform(context)
    generated_mjcf_path = LaunchConfiguration("generated_mjcf_path").perform(context)

    if _is_true(keep_generated_mjcf):
        return [LogInfo(msg=f"[MJCF BUILD] Keeping generated MJCF: {generated_mjcf_path}")]

    if generated_mjcf_path and os.path.exists(generated_mjcf_path):
        os.remove(generated_mjcf_path)
        return [LogInfo(msg=f"[MJCF BUILD] Deleted generated MJCF: {generated_mjcf_path}")]

    return []


def generate_launch_description():
    sim_config_arg = DeclareLaunchArgument(
        "sim_config",
        default_value="none",
        description="Unified YAML config for builder, build resources, output, and simulation settings.",
    )
    builder_python_arg = DeclareLaunchArgument(
        "builder_python",
        default_value="none",
        description="Optional override for builder Python executable.",
    )
    builder_module_arg = DeclareLaunchArgument(
        "builder_module",
        default_value="none",
        description="Optional override for builder Python module.",
    )
    output_mjcf_path_arg = DeclareLaunchArgument(
        "output_mjcf_path",
        default_value="none",
        description="Optional override for generated MJCF output path.",
    )
    keep_generated_mjcf_arg = DeclareLaunchArgument(
        "keep_generated_mjcf",
        default_value="none",
        description="Optional override for keeping generated MJCF after exit.",
    )
    node_class_name_arg = DeclareLaunchArgument(
        "node_class_name",
        default_value="none",
        description="Optional override for simulation node plugin class.",
    )
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value="none",
        description="Optional override for simulation config path.",
    )

    prepare_launch_action = OpaqueFunction(function=_prepare_launch)

    builder_process = ExecuteProcess(
        cmd=[
            LaunchConfiguration("resolved_builder_python"),
            "-m",
            LaunchConfiguration("resolved_builder_module"),
            "--sim-config",
            LaunchConfiguration("sim_config"),
            "--output",
            LaunchConfiguration("generated_mjcf_path"),
        ],
        output="screen",
    )

    simulation_node = Node(
        package="mujoco_sim",
        executable="simulation",
        arguments=[
            LaunchConfiguration("generated_mjcf_path"),
            LaunchConfiguration("resolved_node_class_name"),
            LaunchConfiguration("resolved_config_path"),
        ],
        parameters=[LaunchConfiguration("resolved_config_path")],
        name="simulation",
        output="screen",
    )

    start_simulation_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=builder_process,
            on_exit=lambda event, context: _on_builder_exit(event, context, simulation_node),
        )
    )

    cleanup_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=simulation_node,
            on_exit=[OpaqueFunction(function=_delete_generated_mjcf)],
        )
    )

    return LaunchDescription([
        sim_config_arg,
        builder_python_arg,
        builder_module_arg,
        output_mjcf_path_arg,
        keep_generated_mjcf_arg,
        node_class_name_arg,
        config_path_arg,
        prepare_launch_action,
        builder_process,
        start_simulation_handler,
        cleanup_handler,
    ])
