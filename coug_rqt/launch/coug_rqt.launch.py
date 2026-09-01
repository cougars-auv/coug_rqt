# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
from typing import Any

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def create_diagnostics_config(agent_list: list[str], template_path: str) -> str:
    with open(template_path) as template:
        content = template.read()

    params = yaml.safe_load(content)["diagnostic_aggregator"]["ros__parameters"]
    merged_params = {
        "analyzers": [*agent_list, "base_station"],
        "base_station": params["base_station"],
    }
    for agent_ns in agent_list:
        agent_diagnostics = yaml.safe_load(content.replace("AGENT_NS", agent_ns))
        merged_params[agent_ns] = agent_diagnostics["diagnostic_aggregator"][
            "ros__parameters"
        ][agent_ns]

    with tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".yaml"
    ) as rendered_config:
        yaml.safe_dump(
            {"diagnostic_aggregator": {"ros__parameters": merged_params}},
            rendered_config,
        )
        return rendered_config.name


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Node]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_str = LaunchConfiguration("agent_list").perform(context)

    agent_list = yaml.safe_load(agent_list_str)
    config_dir = os.environ["CONFIG_DIR"]

    fleet_param_file = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "coug_rqt_params.yaml",
        ]
    )
    gui_dir = os.path.join(config_dir, "gui")
    rqt_perspective_file = os.path.join(gui_dir, "rqt.perspective")
    diagnostics_params_file = create_diagnostics_config(
        agent_list, os.path.join(gui_dir, "diagnostic_aggregator.yaml.template")
    )

    return [
        Node(
            package="diagnostic_aggregator",
            executable="aggregator_node",
            name="diagnostic_aggregator",
            parameters=[
                diagnostics_params_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="rqt_gui",
            executable="rqt_gui",
            name="rqt_gui",
            arguments=["--perspective-file", rqt_perspective_file],
            parameters=[
                fleet_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "agent_list": agent_list,
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[auv0]",
                description=(
                    "YAML list of agent namespaces "
                    "(e.g. '[coug1sim]' or '[coug1sim, coug2sim]')"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
