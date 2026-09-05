# Copyright 2026 BYU FROST Lab
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

import math
import os
import threading
from dataclasses import dataclass, field
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from coug_interfaces.srv import BagRecord
from dvl_msgs.msg import ConfigCommand
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import QWidget
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_system_default
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool, Trigger

COLOR_GREEN = "#00cc00"
COLOR_RED = "#cc0000"
COLOR_INFO_TEXT = "#008000"
COLOR_WARN_TEXT = "#808000"
COLOR_ERROR_TEXT = "red"


def _indicator_style(color: str) -> str:
    return f"background-color: {color}; border-radius: 6px;"


@dataclass
class _ServiceCallState:
    service_name: str
    total: int
    responded: int = 0
    succeeded: int = 0
    failed: list[str] = field(default_factory=list)
    response_message: str = ""
    failure_level: str = "warning"


class CougUtilsPlugin(Plugin):
    _gui_call = Signal(object)

    def __init__(self, context: Any) -> None:
        super().__init__(context)
        self.setObjectName("CougUtilsPlugin")
        self._gui_call.connect(lambda callback: callback())

        self._widget = QWidget()
        ui_path = os.path.join(
            get_package_share_directory("coug_rqt"), "ui", "coug_utils.ui"
        )
        loadUi(ui_path, self._widget)
        self._widget.setObjectName("CougUtilsPanelUi")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f"{self._widget.windowTitle()} ({context.serial_number()})"
            )
        context.add_widget(self._widget)

        self._node = context.node
        self._io_node = rclpy.create_node(
            "coug_utils_plugin_node",
            context=self._node.context,
            use_global_arguments=False,
        )
        self._io_executor = SingleThreadedExecutor(context=self._node.context)
        self._io_executor.add_node(self._io_node)
        self._io_thread = threading.Thread(target=self._io_executor.spin, daemon=True)
        self._io_thread.start()

        self._config_command_topic = self._get_or_declare(
            "config_command_topic", "dvl/config/command"
        )
        self._battery_status_topic = self._get_or_declare(
            "battery_status_topic", "battery/status"
        )
        self._bag_record_service = self._get_or_declare(
            "bag_record_service", "bag_record"
        )
        self._arm_thruster_service = self._get_or_declare(
            "arm_thruster_service", "thruster/arm"
        )
        self._emergency_stop_service = self._get_or_declare(
            "emergency_stop_service", "base/emergency_stop"
        )
        self._emergency_surface_service = self._get_or_declare(
            "emergency_surface_service", "base/emergency_surface"
        )
        self._fg_reset_service = self._get_or_declare(
            "fg_reset_service", "factor_graph_node/reset"
        )
        self._depth_calibrate_service = self._get_or_declare(
            "depth_calibrate_service", "depth/calibrate"
        )
        self._fins_calibrate_service = self._get_or_declare(
            "fins_calibrate_service", "fins/calibrate"
        )
        self._agent_list: list[str] = []
        self._service_clients: dict[str, dict[str, Any]] = {}
        self._config_command_pubs: dict[str, Any] = {}
        self._battery_subs: list[Any] = []
        self._battery_voltage_texts: dict[str, str] = {}
        self._indicator_colors: dict[str, dict[QWidget, str]] = {}
        self._current_agent_ns = ""
        self._indicators = (
            self._widget.rosbag_indicator,
            self._widget.armed_indicator,
            self._widget.acoustics_indicator,
        )

        self._widget.agent_selector.currentTextChanged.connect(self._select_agent)
        initial_color = (
            COLOR_GREEN if self._node.get_parameter("use_sim_time").value else COLOR_RED
        )
        for agent_ns in self._get_or_declare("agent_list", [""]):
            if agent_ns:
                self._add_agent(agent_ns, initial_color)
        self._connect_buttons()

    def _get_or_declare(self, name: str, default: Any) -> Any:
        if not self._node.has_parameter(name):
            self._node.declare_parameter(name, default)
        return self._node.get_parameter(name).value

    def _add_agent(self, agent_ns: str, initial_color: str) -> None:
        self._agent_list.append(agent_ns)
        self._battery_voltage_texts[agent_ns] = "Unknown"
        self._service_clients[agent_ns] = {
            self._bag_record_service: self._io_node.create_client(
                BagRecord, f"{agent_ns}/{self._bag_record_service}"
            ),
            self._arm_thruster_service: self._io_node.create_client(
                SetBool, f"{agent_ns}/{self._arm_thruster_service}"
            ),
            self._emergency_stop_service: self._io_node.create_client(
                Trigger, f"{agent_ns}/{self._emergency_stop_service}"
            ),
            self._emergency_surface_service: self._io_node.create_client(
                Trigger, f"{agent_ns}/{self._emergency_surface_service}"
            ),
            self._fg_reset_service: self._io_node.create_client(
                Trigger, f"{agent_ns}/{self._fg_reset_service}"
            ),
            self._depth_calibrate_service: self._io_node.create_client(
                Trigger, f"{agent_ns}/{self._depth_calibrate_service}"
            ),
            self._fins_calibrate_service: self._io_node.create_client(
                Trigger, f"{agent_ns}/{self._fins_calibrate_service}"
            ),
        }
        self._config_command_pubs[agent_ns] = self._io_node.create_publisher(
            ConfigCommand,
            f"{agent_ns}/{self._config_command_topic}",
            qos_profile_system_default,
        )
        self._battery_subs.append(
            self._io_node.create_subscription(
                BatteryState,
                f"{agent_ns}/{self._battery_status_topic}",
                lambda msg, ns=agent_ns: self._battery_status(ns, msg),
                10,
            )
        )
        self._set_indicator(agent_ns, self._widget.armed_indicator, initial_color)
        self._set_indicator(agent_ns, self._widget.acoustics_indicator, initial_color)
        self._widget.agent_selector.addItem(agent_ns)

    def _connect_buttons(self) -> None:
        self._widget.rosbag_start.clicked.connect(lambda: self._record_bag(True))
        self._widget.rosbag_stop.clicked.connect(lambda: self._record_bag(False))
        self._widget.arm_thrusters.clicked.connect(lambda: self._set_armed(True))
        self._widget.disarm_thrusters.clicked.connect(lambda: self._set_armed(False))
        self._widget.enable_dvl_acoustics.clicked.connect(
            lambda: self._set_acoustics(True)
        )
        self._widget.disable_dvl_acoustics.clicked.connect(
            lambda: self._set_acoustics(False)
        )
        self._widget.reset_fg.clicked.connect(
            lambda: self._call_service(self._fg_reset_service, Trigger.Request())
        )
        self._widget.reset_dvl_dr.clicked.connect(
            lambda: self._publish(ConfigCommand(command="reset_dead_reckoning"))
        )
        self._widget.calibrate_depth.clicked.connect(
            lambda: self._call_service(self._depth_calibrate_service, Trigger.Request())
        )
        self._widget.calibrate_fins.clicked.connect(
            lambda: self._call_service(self._fins_calibrate_service, Trigger.Request())
        )
        self._widget.emergency_stop.clicked.connect(
            lambda: self._call_service(self._emergency_stop_service, Trigger.Request())
        )
        self._widget.emergency_surface.clicked.connect(
            lambda: self._call_service(
                self._emergency_surface_service, Trigger.Request()
            )
        )

    def _select_agent(self, agent_ns: str) -> None:
        self._current_agent_ns = agent_ns
        colors = self._indicator_colors.get(agent_ns, {})
        for indicator in self._indicators:
            indicator.setStyleSheet(_indicator_style(colors.get(indicator, COLOR_RED)))
        self._widget.battery_status.setText(
            self._battery_voltage_texts.get(agent_ns, "Unknown")
        )

    def _battery_status(self, agent_ns: str, msg: BatteryState) -> None:
        voltage = msg.voltage
        voltage_text = (
            "Unknown" if math.isnan(voltage) or voltage < 0 else f"{voltage:.2f} V"
        )
        self._battery_voltage_texts[agent_ns] = voltage_text
        if agent_ns == self._current_agent_ns:
            self._gui_call.emit(
                lambda: self._widget.battery_status.setText(voltage_text)
            )

    def _targets(self) -> list[str]:
        if self._widget.apply_all.isChecked():
            if self._agent_list:
                return self._agent_list
            self._status("No agents configured.", "error")
        elif self._current_agent_ns:
            return [self._current_agent_ns]
        else:
            self._status("No agent selected.", "error")
        return []

    def _call_service(
        self,
        service_name: str,
        request: Any,
        indicator: QWidget | None = None,
        color: str | None = None,
    ) -> None:
        targets = self._targets()
        if not targets:
            return
        self._status(f"[{service_name}] Calling service...", "info")
        state = _ServiceCallState(service_name, len(targets))
        for agent_ns in targets:
            client = self._service_clients[agent_ns][service_name]
            if not client.service_is_ready():
                self._record_service_result(
                    state,
                    agent_ns,
                    False,
                    indicator,
                    color,
                    "Service not available: "
                    f"{self._service_name(agent_ns, service_name)}",
                    "error",
                )
                continue
            future = client.call_async(request)
            future.add_done_callback(
                lambda done, ns=agent_ns: self._gui_call.emit(
                    lambda: self._service_response(
                        done, ns, service_name, indicator, color, state
                    )
                )
            )

    def _service_response(
        self,
        future: Any,
        agent_ns: str,
        service_name: str,
        indicator: QWidget | None,
        color: str | None,
        state: _ServiceCallState,
    ) -> None:
        result = None if future.exception() is not None else future.result()
        if result is None:
            self._record_service_result(
                state,
                agent_ns,
                False,
                indicator,
                color,
                f"Service call failed: {self._service_name(agent_ns, service_name)}",
                "error",
            )
            return
        self._record_service_result(
            state,
            agent_ns,
            result.success,
            indicator,
            color,
            result.message,
        )

    def _record_service_result(
        self,
        state: _ServiceCallState,
        agent_ns: str,
        success: bool,
        indicator: QWidget | None,
        color: str | None,
        response_message: str = "",
        failure_level: str = "warning",
    ) -> None:
        if success:
            state.succeeded += 1
            if indicator is not None and color is not None:
                self._set_indicator(agent_ns, indicator, color)
        else:
            state.failed.append(agent_ns)
            state.failure_level = failure_level
        state.response_message = response_message
        state.responded += 1
        if state.responded < state.total:
            return
        if state.total == 1:
            level = "info" if success else state.failure_level
            self._status(
                f"[{state.service_name}] "
                f"{state.response_message or 'Service call completed.'}",
                level,
            )
            return
        if state.succeeded == state.total:
            self._status(
                f"[{state.service_name}] All {state.total} agent(s) confirmed.",
                "info",
            )
            return
        level = "warning" if state.succeeded else "error"
        self._status(
            f"[{state.service_name}] {state.succeeded}/{state.total} confirmed; "
            f"failed: {' '.join(state.failed)}.",
            level,
        )

    def _publish(
        self,
        msg: ConfigCommand,
        indicator: QWidget | None = None,
        color: str | None = None,
    ) -> None:
        targets = self._targets()
        if not targets:
            return
        for agent_ns in targets:
            self._config_command_pubs[agent_ns].publish(msg)
            if indicator is not None and color is not None:
                self._set_indicator(agent_ns, indicator, color)
        self._status(f"Published to {len(targets)} agent(s).", "info")

    def _record_bag(self, start: bool) -> None:
        request = BagRecord.Request()
        request.start = start
        request.prefix = self._widget.bag_prefix.text() if start else ""
        self._call_service(
            self._bag_record_service,
            request,
            self._widget.rosbag_indicator,
            COLOR_GREEN if start else COLOR_RED,
        )

    def _set_armed(self, armed: bool) -> None:
        request = SetBool.Request()
        request.data = armed
        self._call_service(
            self._arm_thruster_service,
            request,
            self._widget.armed_indicator,
            COLOR_GREEN if armed else COLOR_RED,
        )

    def _set_acoustics(self, enabled: bool) -> None:
        msg = ConfigCommand()
        msg.command = "set_config"
        msg.parameter_name = "acoustic_enabled"
        msg.parameter_value = str(enabled).lower()
        self._publish(
            msg,
            self._widget.acoustics_indicator,
            COLOR_GREEN if enabled else COLOR_RED,
        )

    def _set_indicator(self, agent_ns: str, indicator: QWidget, color: str) -> None:
        self._indicator_colors.setdefault(agent_ns, {})[indicator] = color
        if agent_ns == self._current_agent_ns:
            indicator.setStyleSheet(_indicator_style(color))

    def _service_name(self, agent_ns: str, service_name: str) -> str:
        return f"{agent_ns}/{service_name}"

    def _status(self, text: str, level: str) -> None:
        logger = self._node.get_logger()
        if level == "info":
            logger.info(text)
        elif level == "warning":
            logger.warning(text)
        else:
            logger.error(text)
        self._widget.status.setText(text)
        color = {
            "info": COLOR_INFO_TEXT,
            "warning": COLOR_WARN_TEXT,
            "error": COLOR_ERROR_TEXT,
        }[level]
        self._widget.status.setStyleSheet(f"color: {color};")

    def shutdown_plugin(self) -> None:
        self._io_executor.shutdown()
        self._io_thread.join(timeout=1.0)
        self._io_node.destroy_node()

    def save_settings(self, _plugin_settings: Any, _instance_settings: Any) -> None:
        pass

    def restore_settings(self, _plugin_settings: Any, _instance_settings: Any) -> None:
        pass
