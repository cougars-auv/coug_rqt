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
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs) -> list:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_str = LaunchConfiguration("agent_list").perform(context)

    agent_tuples = yaml.safe_load(agent_list_str)
    agent_namespaces = [t[0] for t in agent_tuples]

    pkg_share = get_package_share_directory("coug_rqt")

    rqt_perspective_file = os.path.join(pkg_share, "rqt", "rqt.perspective")

    template_path = os.path.join(
        pkg_share, "diagnostics", "diagnostics_params.yaml.template"
    )
    with open(template_path, "r") as f:
        template_content = f.read()

    merged_params = {"analyzers": agent_namespaces}
    for ns in agent_namespaces:
        agent_yaml = yaml.safe_load(template_content.replace("AUV_NS", ns))
        merged_params[ns] = agent_yaml["diagnostic_aggregator"]["ros__parameters"][ns]

    merged_config = {"diagnostic_aggregator": {"ros__parameters": merged_params}}

    temp_config = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".yaml")
    yaml.safe_dump(merged_config, temp_config)
    temp_config.close()

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
            parameters=[{"use_sim_time": use_sim_time}],
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
                default_value="[[auv0, bluerov2.urdf.xacro]]",
                description=(
                    "YAML list of [auv_ns, auv_urdf] pairs "
                    "(e.g. '[[coug0sim, couguv_holoocean.urdf.xacro], "
                    "[coug1sim, couguv_holoocean.urdf.xacro]]')"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
