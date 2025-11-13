"""Launch file that mimics running scripts/walk_test.py directly."""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    declared_arguments = [
        DeclareLaunchArgument(
            "onnx_model_path",
            default_value="/home/bdxv2/Open_Duck_Mini/BEST_WALK_ONNX_2.onnx",
            description="Path to the ONNX policy file passed to walk_test.py",
        ),
        DeclareLaunchArgument(
            "duck_config_path",
            default_value=os.path.expanduser("~/duck_config.json"),
            description="Duck configuration JSON file",
        ),
        DeclareLaunchArgument(
            "action_scale",
            default_value="0.25",
            description="Action scaling factor",
        ),
        DeclareLaunchArgument(
            "leg_action_multiplier",
            default_value="1.0",
            description="Extra multiplier applied to leg joints",
        ),
        DeclareLaunchArgument("p", default_value="30", description="P gain"),
        DeclareLaunchArgument("i", default_value="0", description="I gain"),
        DeclareLaunchArgument("d", default_value="0", description="D gain"),
        DeclareLaunchArgument(
            "control_freq",
            default_value="50",
            description="Control frequency in Hz",
        ),
        DeclareLaunchArgument(
            "pitch_bias",
            default_value="0",
            description="Pitch bias in degrees",
        ),
        DeclareLaunchArgument(
            "hip_pitch_trim_deg",
            default_value="0.0",
            description="Hip pitch trim to counter lean",
        ),
        DeclareLaunchArgument(
            "save_obs",
            default_value="",
            description="File path to save observations (leave empty to disable)",
        ),
        DeclareLaunchArgument(
            "replay_obs",
            default_value="",
            description="Observation log to replay (leave empty for live data)",
        ),
        DeclareLaunchArgument(
            "cutoff_frequency",
            default_value="",
            description="Cutoff frequency for the action filter",
        ),
        DeclareLaunchArgument(
            "disable_imu_auto_tare",
            default_value="false",
            description="Set to true to skip the automatic IMU tare calibration",
        ),
        DeclareLaunchArgument(
            "imu_tare_window",
            default_value="120",
            description="Samples used during IMU tare calibration",
        ),
        DeclareLaunchArgument(
            "imu_tare_std_threshold",
            default_value="0.05",
            description="Std-dev threshold used for IMU tare calibration",
        ),
    ]

    def launch_setup(context, *_) -> list:
        def perform(name: str) -> str:
            return LaunchConfiguration(name).perform(context)

        cmd = [
            "--onnx_model_path",
            perform("onnx_model_path"),
            "--duck_config_path",
            perform("duck_config_path"),
            "-p",
            perform("p"),
            "-i",
            perform("i"),
            "-d",
            perform("d"),
            "-c",
            perform("control_freq"),
            "-a",
            perform("action_scale"),
            "--leg_action_multiplier",
            perform("leg_action_multiplier"),
            "--pitch_bias",
            perform("pitch_bias"),
            "--hip_pitch_trim_deg",
            perform("hip_pitch_trim_deg"),
            "--imu_tare_window",
            perform("imu_tare_window"),
            "--imu_tare_std_threshold",
            perform("imu_tare_std_threshold"),
        ]

        save_obs = perform("save_obs")
        if save_obs:
            cmd.extend(["--save_obs", save_obs])

        replay_obs = perform("replay_obs")
        if replay_obs:
            cmd.extend(["--replay_obs", replay_obs])

        cutoff = perform("cutoff_frequency")
        if cutoff:
            cmd.extend(["--cutoff_frequency", cutoff])

        disable_tare = perform("disable_imu_auto_tare").lower()
        if disable_tare in {"1", "true", "on"}:
            cmd.append("--disable_imu_auto_tare")

        node = Node(
            package="duck_walk_bringup",
            executable="walk",
            name="duck_walk_runtime",
            output="screen",
            arguments=cmd,
            emulate_tty=True,
        )
        return [node]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
