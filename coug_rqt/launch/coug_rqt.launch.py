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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Node]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_str = LaunchConfiguration("agent_list").perform(context)

    agent_list = yaml.safe_load(agent_list_str)

    pkg_share = get_package_share_directory("coug_rqt")
    fleet_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "coug_rqt_params.yaml",
        ]
    )
    rqt_perspective_file = os.path.join(pkg_share, "config", "rqt_config.perspective")
    template_path = os.path.join(
        pkg_share, "config", "diagnostic_aggregator_params.yaml.template"
    )

    with open(template_path, "r") as f:
        template_content = f.read()

    def analyzer(content: str, key: str) -> dict[str, Any]:
        params = yaml.safe_load(content)["diagnostic_aggregator"]["ros__parameters"]
        return params[key]

    merged_params = {"analyzers": agent_list + ["base_station"]}
    for ns in agent_list:
        merged_params[ns] = analyzer(template_content.replace("AGENT_NS", ns), ns)
    merged_params["base_station"] = analyzer(template_content, "base_station")

    merged_config = {"diagnostic_aggregator": {"ros__parameters": merged_params}}

    with tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".yaml"
    ) as temp_config:
        yaml.safe_dump(merged_config, temp_config)
        diagnostics_params_file = temp_config.name

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
                fleet_params,
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
