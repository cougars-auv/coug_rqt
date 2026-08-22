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

import math
import os
import threading
from collections.abc import Callable
from typing import Any

import rclpy
import rclpy.exceptions
from ament_index_python.packages import get_package_share_directory
from coug_interfaces.srv import BagRecord
from dvl_msgs.msg import ConfigCommand
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import QMessageBox, QWidget
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_system_default
from rclpy.task import Future
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool, Trigger

COLOR_GREEN = "#00cc00"
COLOR_RED = "#cc0000"


class CougUtilsPlugin(Plugin):
    _deliver = Signal(object)

    def __init__(self, context: Any) -> None:
        super().__init__(context)
        self.setObjectName("CougUtilsPlugin")
        self._deliver.connect(self._run_on_gui_thread)

        self._widget = QWidget()
        loadUi(
            os.path.join(
                get_package_share_directory("coug_rqt"), "ui", "coug_utils.ui"
            ),
            self._widget,
        )
        self._widget.setObjectName("CougUtilsPanelUi")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f"{self._widget.windowTitle()} ({context.serial_number()})"
            )
        context.add_widget(self._widget)

        self._node = context.node

        self._srv_node = rclpy.create_node(
            "coug_utils_plugin_node",
            context=context.node.context,
            use_global_arguments=False,  # IMPORTANT! Disable node name override
        )
        self._srv_executor = SingleThreadedExecutor(context=context.node.context)
        self._srv_executor.add_node(self._srv_node)
        self._srv_thread = threading.Thread(target=self._srv_executor.spin, daemon=True)
        self._srv_thread.start()

        try:
            self._node.declare_parameter("agent_namespaces", [""])
            self._node.declare_parameter("config_command_topic", "dvl/config/command")
            self._node.declare_parameter("battery_status_topic", "battery/status")
            self._node.declare_parameter("bag_record_service", "bag_record")
            self._node.declare_parameter("arm_thruster_service", "thruster/arm")
            self._node.declare_parameter(
                "emergency_stop_service", "base/emergency_stop"
            )
            self._node.declare_parameter(
                "emergency_surface_service", "base/emergency_surface"
            )
            self._node.declare_parameter("fgo_reset_service", "factor_graph_node/reset")
            self._node.declare_parameter("depth_calibrate_service", "depth/calibrate")
            self._node.declare_parameter("fins_calibrate_service", "fins/calibrate")
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self._agent_namespaces = []
        self._clients = {}
        self._pubs = {}
        self._battery_subs = {}
        self._battery_voltages = {}
        self._current_agent = ""
        self._agent_state = {}
        self._indicators = [
            self._widget.rosbag_indicator,
            self._widget.armed_indicator,
            self._widget.acoustics_indicator,
        ]
        self._config_command_topic = self._node.get_parameter(
            "config_command_topic"
        ).value
        self._battery_status_topic = self._node.get_parameter(
            "battery_status_topic"
        ).value
        self._bag_record_service = self._node.get_parameter("bag_record_service").value
        self._arm_thruster_srv = self._node.get_parameter("arm_thruster_service").value
        self._emergency_stop_service = self._node.get_parameter(
            "emergency_stop_service"
        ).value
        self._emergency_surface_service = self._node.get_parameter(
            "emergency_surface_service"
        ).value
        self._fgo_reset_service = self._node.get_parameter("fgo_reset_service").value
        self._depth_calibrate_service = self._node.get_parameter(
            "depth_calibrate_service"
        ).value
        self._fins_calibrate_service = self._node.get_parameter(
            "fins_calibrate_service"
        ).value

        # In sim, thrusters and acoustics start enabled (green); otherwise off (red).
        sim_color = (
            COLOR_GREEN if self._node.get_parameter("use_sim_time").value else COLOR_RED
        )

        self._widget.agent_selector.currentTextChanged.connect(self._on_agent_changed)
        for ns in self._node.get_parameter("agent_namespaces").value:
            if ns:
                self._agent_namespaces.append(ns)
                self._battery_voltages[ns] = "Unknown"
                self._clients[ns] = {
                    self._bag_record_service: self._srv_node.create_client(
                        BagRecord, f"{ns}/{self._bag_record_service}"
                    ),
                    self._arm_thruster_srv: self._srv_node.create_client(
                        SetBool, f"{ns}/{self._arm_thruster_srv}"
                    ),
                    self._emergency_stop_service: self._srv_node.create_client(
                        Trigger, f"{ns}/{self._emergency_stop_service}"
                    ),
                    self._emergency_surface_service: self._srv_node.create_client(
                        Trigger, f"{ns}/{self._emergency_surface_service}"
                    ),
                    self._fgo_reset_service: self._srv_node.create_client(
                        Trigger, f"{ns}/{self._fgo_reset_service}"
                    ),
                    self._depth_calibrate_service: self._srv_node.create_client(
                        Trigger, f"{ns}/{self._depth_calibrate_service}"
                    ),
                    self._fins_calibrate_service: self._srv_node.create_client(
                        Trigger, f"{ns}/{self._fins_calibrate_service}"
                    ),
                }
                self._pubs[ns] = self._srv_node.create_publisher(
                    ConfigCommand,
                    f"{ns}/{self._config_command_topic}",
                    qos_profile_system_default,
                )
                self._battery_subs[ns] = self._srv_node.create_subscription(
                    BatteryState,
                    f"{ns}/{self._battery_status_topic}",
                    lambda msg, agent_ns=ns: self._on_battery_status(msg, agent_ns),
                    10,
                )
                self._set_indicator(ns, self._widget.armed_indicator, sim_color)
                self._set_indicator(ns, self._widget.acoustics_indicator, sim_color)
                self._widget.agent_selector.addItem(ns)

        self._widget.rosbag_start.clicked.connect(self._rosbag_start)
        self._widget.rosbag_stop.clicked.connect(self._rosbag_stop)
        self._widget.arm_thrusters.clicked.connect(self._arm_thrusters)
        self._widget.disarm_thrusters.clicked.connect(self._disarm_thrusters)
        self._widget.enable_dvl_acoustics.clicked.connect(self._enable_dvl_acoustics)
        self._widget.disable_dvl_acoustics.clicked.connect(self._disable_dvl_acoustics)
        self._widget.reset_fgo.clicked.connect(self._reset_fgo)
        self._widget.reset_dvl_dr.clicked.connect(self._reset_dvl_dr)
        self._widget.calibrate_depth.clicked.connect(self._calibrate_depth)
        self._widget.calibrate_fins.clicked.connect(self._calibrate_fins)
        self._widget.emergency_stop.clicked.connect(self._emergency_stop)
        self._widget.emergency_surface.clicked.connect(self._emergency_surface)

    def _on_agent_changed(self, text: str) -> None:
        self._current_agent = text
        state = self._agent_state.get(text, {})
        for indicator in self._indicators:
            color = state.get(indicator, COLOR_RED)
            indicator.setStyleSheet(f"background-color: {color}; border-radius: 6px;")

        voltage = self._battery_voltages.get(text, "Unknown")
        self._widget.battery_status.setText(voltage)

    def _on_battery_status(self, msg: BatteryState, agent_ns: str) -> None:
        val = msg.voltage
        txt = "Unknown" if math.isnan(val) or val < 0.0 else f"{val:.2f} V"
        self._battery_voltages[agent_ns] = txt
        if agent_ns == self._current_agent:
            self._deliver.emit(lambda: self._widget.battery_status.setText(txt))

    def _set_indicator(self, ns: str, indicator: QWidget | None, color: str) -> None:
        if indicator is None:
            return
        self._agent_state.setdefault(ns, {})[indicator] = color
        if ns == self._current_agent:
            indicator.setStyleSheet(f"background-color: {color}; border-radius: 6px;")

    def _print_info(self, text: str) -> None:
        self._node.get_logger().info(text)
        self._widget.status.setText(text)
        self._widget.status.setStyleSheet("color: #008000;")

    def _print_warn(self, text: str) -> None:
        self._node.get_logger().warning(text)
        self._widget.status.setText(text)
        self._widget.status.setStyleSheet("color: #808000;")

    def _print_error(self, text: str) -> None:
        self._node.get_logger().error(text)
        self._widget.status.setText(text)
        self._widget.status.setStyleSheet("color: red;")

    def _confirm(self, action: str) -> bool:
        if self._widget.apply_all.isChecked():
            count = len(self._agent_namespaces)
            target = f"all {count} agent(s)" if count else ""
        else:
            target = self._current_agent
        if not target:
            return True

        reply = QMessageBox.question(
            self._widget,
            "CoUGARs Utils",
            f"{action} on {target}?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        return reply == QMessageBox.Yes

    def _call_service(
        self,
        service_name: str,
        request: Any,
        indicator: QWidget | None,
        color: str | None,
    ) -> None:
        if self._widget.apply_all.isChecked():
            if not self._agent_namespaces:
                self._print_error("No agents configured.")
                return
            self._print_info(f"[{service_name}] Calling service...")
            state = {
                "cmd": service_name,
                "total": len(self._agent_namespaces),
                "responded": 0,
                "succeeded": 0,
                "failed": [],
            }
            for ns in self._agent_namespaces:
                self._call_agent_service(
                    ns, service_name, request, indicator, color, state
                )
        elif self._current_agent:
            self._print_info(f"[{service_name}] Calling service...")
            self._call_agent_service(
                self._current_agent, service_name, request, indicator, color
            )
        else:
            self._print_error("No agent selected.")

    def _publish_topic(
        self, msg: Any, indicator: QWidget | None, color: str | None
    ) -> None:
        if self._widget.apply_all.isChecked():
            if not self._agent_namespaces:
                self._print_error("No agents configured.")
                return
            for ns in self._agent_namespaces:
                self._pubs[ns].publish(msg)
                self._set_indicator(ns, indicator, color)
            self._print_info(f"Published {len(self._agent_namespaces)} agent(s).")
        elif self._current_agent:
            self._pubs[self._current_agent].publish(msg)
            self._set_indicator(self._current_agent, indicator, color)
            self._print_info("Published 1 agent(s).")
        else:
            self._print_error("No agent selected.")

    def _run_on_gui_thread(self, fn: Callable[[], None]) -> None:
        fn()

    def _call_agent_service(
        self,
        ns: str,
        service_name: str,
        request: Any,
        indicator: QWidget | None,
        color: str | None,
        state: dict[str, Any] | None = None,
    ) -> None:
        client = self._clients.get(ns, {}).get(service_name)
        if client is None or not client.service_is_ready():
            if state is not None:
                self._deliver.emit(
                    lambda: self._record_result(state, ns, False, indicator, color)
                )
            else:
                self._print_error(f"Service not available: {ns}/{service_name}")
            return

        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self._deliver.emit(
                lambda: self._on_response(f, ns, service_name, indicator, color, state)
            )
        )

    def _on_response(
        self,
        future: Future,
        ns: str,
        service_name: str,
        indicator: QWidget | None,
        color: str | None,
        state: dict[str, Any] | None,
    ) -> None:
        result = future.result()
        success = result is not None and result.success
        if state is not None:
            self._record_result(state, ns, success, indicator, color)
        elif success:
            self._set_indicator(ns, indicator, color)
            self._print_info(f"[{service_name}] {result.message}")
        elif result is not None:
            self._print_warn(f"[{service_name}] {result.message}")
        else:
            self._print_error("Service call failed (no response).")

    def _record_result(
        self,
        state: dict[str, Any],
        ns: str,
        success: bool,
        indicator: QWidget | None,
        color: str | None,
    ) -> None:
        if success:
            state["succeeded"] += 1
            self._set_indicator(ns, indicator, color)
        else:
            state["failed"].append(ns)
        state["responded"] += 1
        if state["responded"] < state["total"]:
            return
        s, t, cmd = state["succeeded"], state["total"], state["cmd"]
        if s == t:
            self._print_info(f"[{cmd}] All {t} agent(s) confirmed.")
        elif s > 0:
            self._print_warn(
                f"[{cmd}] {s}/{t} confirmed; failed: {' '.join(state['failed'])}."
            )
        else:
            self._print_error(
                f"[{cmd}] 0/{t} confirmed; failed: {' '.join(state['failed'])}."
            )

    def _rosbag_start(self) -> None:
        req = BagRecord.Request()
        req.start = True
        req.prefix = self._widget.bag_prefix.text()
        self._call_service(
            self._bag_record_service, req, self._widget.rosbag_indicator, COLOR_GREEN
        )

    def _rosbag_stop(self) -> None:
        req = BagRecord.Request()
        req.start = False
        req.prefix = ""
        self._call_service(
            self._bag_record_service, req, self._widget.rosbag_indicator, COLOR_RED
        )

    def _arm_thrusters(self) -> None:
        req = SetBool.Request()
        req.data = True
        self._call_service(
            self._arm_thruster_srv, req, self._widget.armed_indicator, COLOR_GREEN
        )

    def _disarm_thrusters(self) -> None:
        req = SetBool.Request()
        req.data = False
        self._call_service(
            self._arm_thruster_srv, req, self._widget.armed_indicator, COLOR_RED
        )

    def _acoustics_command(self, enabled: bool) -> ConfigCommand:
        msg = ConfigCommand()
        msg.command = "set_config"
        msg.parameter_name = "acoustic_enabled"
        msg.parameter_value = "true" if enabled else "false"
        return msg

    def _enable_dvl_acoustics(self) -> None:
        self._publish_topic(
            self._acoustics_command(True),
            self._widget.acoustics_indicator,
            COLOR_GREEN,
        )

    def _disable_dvl_acoustics(self) -> None:
        self._publish_topic(
            self._acoustics_command(False),
            self._widget.acoustics_indicator,
            COLOR_RED,
        )

    def _reset_fgo(self) -> None:
        if not self._confirm("Reset FGO"):
            return
        self._call_service(
            self._fgo_reset_service,
            Trigger.Request(),
            None,
            None,
        )

    def _reset_dvl_dr(self) -> None:
        if not self._confirm("Reset DVL DR"):
            return
        msg = ConfigCommand()
        msg.command = "reset_dead_reckoning"
        self._publish_topic(msg, None, None)

    def _calibrate_depth(self) -> None:
        self._call_service(
            self._depth_calibrate_service,
            Trigger.Request(),
            None,
            None,
        )

    def _calibrate_fins(self) -> None:
        self._call_service(
            self._fins_calibrate_service,
            Trigger.Request(),
            None,
            None,
        )

    def _emergency_stop(self) -> None:
        self._call_service(self._emergency_stop_service, Trigger.Request(), None, None)

    def _emergency_surface(self) -> None:
        self._call_service(
            self._emergency_surface_service, Trigger.Request(), None, None
        )

    def shutdown_plugin(self) -> None:
        for ns_clients in self._clients.values():
            for client in ns_clients.values():
                self._srv_node.destroy_client(client)
        self._clients = {}

        for publisher in self._pubs.values():
            self._srv_node.destroy_publisher(publisher)
        self._pubs = {}

        for sub in self._battery_subs.values():
            self._srv_node.destroy_subscription(sub)
        self._battery_subs = {}

        self._srv_executor.shutdown()
        self._srv_thread.join(timeout=1.0)
        self._srv_node.destroy_node()

    def save_settings(self, _plugin_settings: Any, _instance_settings: Any) -> None:
        pass

    def restore_settings(self, _plugin_settings: Any, _instance_settings: Any) -> None:
        pass
