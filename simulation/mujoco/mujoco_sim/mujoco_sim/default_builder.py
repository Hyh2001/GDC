#!/usr/bin/env python3

import argparse
import sys

from mujoco_sim.builder_utils import (
    default_postprocess_mjcf,
    default_resolve_resources,
    load_spec_with_assets,
    load_yaml_file,
)


def build_mjcf_with_scene(robot_path, scene_path):
    robot_spec, robot_asset_paths = load_spec_with_assets(robot_path, exported_prefix="robot_")
    scene_spec, scene_asset_paths = load_spec_with_assets(scene_path)

    attach_frame = scene_spec.worldbody.add_frame()
    scene_spec.attach(robot_spec, "robot_", "", frame=attach_frame)
    scene_spec.assets = {**scene_spec.assets, **robot_spec.assets}

    scene_spec.compile()
    mjcf_content = scene_spec.to_xml()
    asset_paths = {
        **scene_asset_paths,
        **robot_asset_paths,
    }
    return default_postprocess_mjcf(mjcf_content, asset_paths)


def build_mjcf_without_scene(robot_path):
    robot_spec, asset_paths = load_spec_with_assets(robot_path)
    robot_spec.compile()
    mjcf_content = robot_spec.to_xml()
    return default_postprocess_mjcf(mjcf_content, asset_paths)


def resolve_resources(config):
    resolved, robot_resource = default_resolve_resources(config)
    scene_resource = next(
        (resource for resource in resolved if resource.get("name") == "scene"),
        None,
    )
    if scene_resource is not None:
        mjcf_content = build_mjcf_with_scene(robot_resource["path"], scene_resource["path"])
    else:
        mjcf_content = build_mjcf_without_scene(robot_resource["path"])

    return mjcf_content

def main():
    parser = argparse.ArgumentParser(description="Default MJCF builder.")
    parser.add_argument("--sim-config", required=True, help="Unified sim config YAML path.")
    parser.add_argument("--output", required=True, help="Output MJCF file path.")
    args = parser.parse_args()

    config = load_yaml_file(args.sim_config)
    mjcf_content = resolve_resources(config)

    with open(args.output, "w") as f:
        f.write(mjcf_content)

    print(f"[MJCF BUILD] Wrote default MJCF to: {args.output}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[MJCF BUILD] Error: {exc}", file=sys.stderr)
        sys.exit(1)
