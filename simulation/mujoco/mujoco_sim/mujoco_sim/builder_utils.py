import os
import posixpath
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import yaml
from ament_index_python.packages import get_package_share_path


def load_yaml_file(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML config must contain a dictionary at top level: {path}")
    return data


def resolve_resource_path(resource):
    if not isinstance(resource, dict):
        raise RuntimeError("Each resource must be a dictionary.")

    package = resource.get("package")
    path = resource.get("path")
    name = resource.get("name", "<unnamed>")

    if not package:
        raise RuntimeError(f"Resource {name} is missing `package`.")
    if not path:
        raise RuntimeError(f"Resource {name} is missing `path`.")

    return os.path.join(get_package_share_path(package), path)


def _resolve_asset_file(xml_dir, compiler_dir, file_attr):
    asset_path = Path(file_attr)
    if asset_path.is_absolute():
        return asset_path
    return (xml_dir / compiler_dir / asset_path).resolve()


def _normalize_asset_alias(path_str):
    normalized = posixpath.normpath(path_str)
    return "." if normalized == "" else normalized


def load_spec_with_assets(xml_path, exported_prefix=""):
    spec = mujoco.MjSpec.from_file(str(xml_path))

    xml_path = Path(xml_path).resolve()
    root = ET.parse(xml_path).getroot()

    compiler = root.find("compiler")
    meshdir = Path(compiler.get("meshdir", ".")) if compiler is not None else Path(".")
    texturedir = Path(compiler.get("texturedir", ".")) if compiler is not None else Path(".")

    assets = {}
    asset_paths = {}
    asset_root = root.find("asset")
    if asset_root is not None:
        for asset in asset_root:
            file_attr = asset.get("file")
            if not file_attr:
                continue

            tag = asset.tag
            compiler_dir = texturedir if tag == "texture" else meshdir
            full_path = _resolve_asset_file(xml_path.parent, compiler_dir, file_attr)
            if not full_path.is_file():
                raise RuntimeError(f"Referenced asset file does not exist: {full_path}")

            file_bytes = full_path.read_bytes()
            asset_keys = {_normalize_asset_alias(file_attr)}
            asset_paths[_normalize_asset_alias(file_attr)] = str(full_path)

            if not Path(file_attr).is_absolute():
                relative_path = Path(file_attr)
                compiler_relative = _normalize_asset_alias(str((compiler_dir / relative_path).as_posix()))
                asset_keys.add(compiler_relative)
                asset_paths[compiler_relative] = str(full_path)

                if exported_prefix:
                    prefixed_relative = relative_path.parent / f"{exported_prefix}{relative_path.name}"
                    asset_paths[_normalize_asset_alias(str(prefixed_relative.as_posix()))] = str(full_path)
                    asset_paths[_normalize_asset_alias(str((compiler_dir / prefixed_relative).as_posix()))] = str(full_path)

            for asset_key in asset_keys:
                assets[asset_key] = file_bytes

    spec.assets = assets
    return spec, asset_paths


def default_postprocess_mjcf(mjcf_content, asset_paths, prefix="robot_"):
    root = ET.fromstring(mjcf_content)

    asset_root = root.find("asset")
    if asset_root is not None:
        for asset in asset_root:
            file_attr = asset.get("file")
            if not file_attr:
                continue
            resolved_path = asset_paths.get(_normalize_asset_alias(file_attr))
            if resolved_path is not None:
                asset.set("file", resolved_path)

    def strip_prefix(value):
        if value and value.startswith(prefix):
            return value[len(prefix):]
        return value

    for joint in root.findall(".//joint"):
        name = joint.get("name")
        if name:
            joint.set("name", strip_prefix(name))

    actuator_root = root.find("actuator")
    if actuator_root is not None:
        for actuator in actuator_root:
            name = actuator.get("name")
            if name:
                actuator.set("name", strip_prefix(name))

            joint_name = actuator.get("joint")
            if joint_name:
                actuator.set("joint", strip_prefix(joint_name))

    sensor_root = root.find("sensor")
    if sensor_root is not None:
        for sensor in sensor_root:
            name = sensor.get("name")
            if name:
                sensor.set("name", strip_prefix(name))

            joint_name = sensor.get("joint")
            if joint_name:
                sensor.set("joint", strip_prefix(joint_name))

    equality_root = root.find("equality")
    if equality_root is not None:
        for equality in equality_root:
            joint1 = equality.get("joint1")
            if joint1:
                equality.set("joint1", strip_prefix(joint1))

            joint2 = equality.get("joint2")
            if joint2:
                equality.set("joint2", strip_prefix(joint2))

    keyframe_root = root.find("keyframe")
    if keyframe_root is not None:
        for key in keyframe_root.findall("key"):
            name = key.get("name")
            if name == f"{prefix}home":
                key.set("name", "home")

    return ET.tostring(root, encoding="unicode")


def default_resolve_resources(config):
    build_config = config.get("build", {})
    if not isinstance(build_config, dict):
        raise RuntimeError("`build` section must be a dictionary.")
    resources = build_config.get("resources")
    if not isinstance(resources, list) or not resources:
        raise RuntimeError("`build.resources` must be a non-empty list for default builder.")

    resolved = []
    for resource in resources:
        resolved.append(
            {
                "name": resource.get("name", ""),
                "path": resolve_resource_path(resource),
            }
        )

    robot_resource = next(
        (resource for resource in resolved if resource.get("name") == "robot"),
        None,
    )
    if robot_resource is None:
        raise RuntimeError("`build.resources` must contain a resolved resource with name `robot`.")

    return resolved, robot_resource
