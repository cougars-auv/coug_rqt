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

import rclpy.exceptions
from ament_index_python.packages import get_package_share_directory
from coug_interfaces.srv import BagRecord
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin


class CougUtilsPlugin(Plugin):
    """
    RQT panel for per-agent utility commands.

    :author: Nelson Durrant
    :date: June 2026
    """

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("CougUtilsPlugin")

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
                self._widget.windowTitle() + " (%d)" % context.serial_number()
            )
        context.add_widget(self._widget)

        self._node = context.node
        try:
            self._node.declare_parameter("agent_namespaces", [""])
            self._node.declare_parameter("bag_record_service", "bag_record")
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self._agent_namespaces = []
        self._clients = {}
        self._current_agent = ""
        self._agent_state = {}
        self._indicators = [
            self._widget.rosbag_indicator,
            self._widget.armed_indicator,
            self._widget.acoustics_indicator,
        ]
        self._bag_record_service = self._node.get_parameter("bag_record_service").value

        self._widget.agent_selector.currentTextChanged.connect(self._on_agent_changed)
        for ns in self._node.get_parameter("agent_namespaces").value:
            if ns:
                self._agent_namespaces.append(ns)
                self._clients[ns] = {
                    self._bag_record_service: self._node.create_client(
                        BagRecord, f"{ns}/{self._bag_record_service}"
                    ),
                }
                self._widget.agent_selector.addItem(ns)

        self._widget.rosbag_start.clicked.connect(self._rosbag_start)
        self._widget.rosbag_stop.clicked.connect(self._rosbag_stop)
        self._widget.arm.clicked.connect(self._arm)
        self._widget.disarm.clicked.connect(self._disarm)
        self._widget.acoustics_on.clicked.connect(self._acoustics_on)
        self._widget.acoustics_off.clicked.connect(self._acoustics_off)
        self._widget.emergency_stop.clicked.connect(self._emergency_stop)
        self._widget.emergency_surface.clicked.connect(self._emergency_surface)

    def _on_agent_changed(self, text):
        """
        Handle change in selected agent text.

        :param text: The selected agent namespace.
        """
        self._current_agent = text
        state = self._agent_state.get(text, {})
        for indicator in self._indicators:
            color = state.get(indicator, "#cc0000")
            indicator.setStyleSheet(f"background-color: {color}; border-radius: 6px;")

    def _set_indicator(self, ns, indicator, color):
        """
        Set the visual indicator color for a specific agent namespace.

        :param ns: Agent namespace.
        :param indicator: Indicator widget reference.
        :param color: Hex color string.
        """
        self._agent_state.setdefault(ns, {})[indicator] = color
        if ns == self._current_agent:
            indicator.setStyleSheet(f"background-color: {color}; border-radius: 6px;")

    def _print_info(self, text):
        """
        Print informational log message and update panel status.

        :param text: Information message string.
        """
        self._node.get_logger().info(text)
        self._widget.status.setText(text)
        self._widget.status.setStyleSheet("color: #008000;")

    def _print_warn(self, text):
        """
        Print warning log message and update panel status.

        :param text: Warning message string.
        """
        self._node.get_logger().warning(text)
        self._widget.status.setText(text)
        self._widget.status.setStyleSheet("color: #808000;")

    def _print_error(self, text):
        """
        Print error log message and update panel status.

        :param text: Error message string.
        """
        self._node.get_logger().error(text)
        self._widget.status.setText(text)
        self._widget.status.setStyleSheet("color: red;")

    def _call_service(self, service_name, request, indicator, color, on_success=None):
        """
        Call a service on either the currently selected agent or all agents.

        :param service_name: Name of the service to call.
        :param request: Service request message.
        :param indicator: Visual indicator widget.
        :param color: Visual indicator target success color.
        :param on_success: Optional callback function upon successful response.
        """
        if on_success is None:
            on_success = self._print_info
        if self._widget.apply_all.isChecked():
            if not self._agent_namespaces:
                self._print_error("No agents configured")
                return
            self._print_info(f"[{service_name}] calling...")
            state = {
                "cmd": service_name,
                "total": len(self._agent_namespaces),
                "responded": 0,
                "succeeded": 0,
                "failed": [],
                "on_success": on_success,
            }
            for ns in self._agent_namespaces:
                self._dispatch(ns, service_name, request, indicator, color, state)
        elif self._current_agent:
            self._print_info(f"[{service_name}] calling...")
            self._dispatch(
                self._current_agent,
                service_name,
                request,
                indicator,
                color,
                on_success=on_success,
            )
        else:
            self._print_error("No agent selected")

    def _dispatch(
        self, ns, service_name, request, indicator, color, state=None, on_success=None
    ):
        """
        Dispatch a service call to a specific agent namespace asynchronously.

        :param ns: Agent namespace.
        :param service_name: Name of the service.
        :param request: Service request message.
        :param indicator: Visual indicator widget.
        :param color: Visual indicator target success color.
        :param state: Dispatch state tracking dict for multi-agent calls.
        :param on_success: Callback function upon successful response.
        """
        client = self._clients.get(ns, {}).get(service_name)
        if client is None or not client.service_is_ready():
            if state is not None:
                QTimer.singleShot(
                    0, lambda: self._record_result(state, ns, False, indicator, color)
                )
            else:
                self._print_error(f"Service not available: {ns}/{service_name}")
            return
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: QTimer.singleShot(
                0,
                lambda: self._on_response(
                    f, ns, service_name, indicator, color, state, on_success
                ),
            )
        )

    def _on_response(
        self, future, ns, service_name, indicator, color, state, on_success
    ):
        """
        Callback triggered when a service call future completes.

        :param future: The completed future object.
        :param ns: Agent namespace.
        :param service_name: Name of the service.
        :param indicator: Visual indicator widget.
        :param color: Visual indicator target success color.
        :param state: Dispatch state tracking dict.
        :param on_success: Callback function upon successful response.
        """
        result = future.result()
        success = result is not None and result.success
        if state is not None:
            self._record_result(state, ns, success, indicator, color)
        elif success:
            self._set_indicator(ns, indicator, color)
            on_success(f"[{service_name}] {result.message}")
        elif result is not None:
            self._print_warn(f"[{service_name}] {result.message}")
        else:
            self._print_error("Service call failed (no response)")

    def _record_result(self, state, ns, success, indicator, color):
        """
        Record the success or failure result of a dispatched service call.

        :param state: Dispatch state tracking dict.
        :param ns: Agent namespace.
        :param success: Boolean flag indicating success.
        :param indicator: Visual indicator widget.
        :param color: Visual indicator target success color.
        """
        if success:
            state["succeeded"] += 1
            self._set_indicator(ns, indicator, color)
        else:
            state["failed"].append(ns)
        state["responded"] += 1
        if state["responded"] < state["total"]:
            return
        s, t, cmd = state["succeeded"], state["total"], state["cmd"]
        on_success = state["on_success"]
        if s == t:
            on_success(f"[{cmd}] All {t} agent(s) sent")
        elif s > 0:
            self._print_warn(
                f"[{cmd}] {s}/{t} confirmed; failed: {' '.join(state['failed'])}"
            )
        else:
            self._print_error(
                f"[{cmd}] 0/{t} confirmed; failed: {' '.join(state['failed'])}"
            )

    def _rosbag_start(self):
        """
        Trigger the start of rosbag recording.
        """
        req = BagRecord.Request()
        req.start = True
        req.prefix = self._widget.bag_prefix.text()
        self._call_service(
            self._bag_record_service, req, self._widget.rosbag_indicator, "#00cc00"
        )

    def _rosbag_stop(self):
        """
        Trigger the stop of rosbag recording.
        """
        req = BagRecord.Request()
        req.start = False
        req.prefix = ""
        self._call_service(
            self._bag_record_service,
            req,
            self._widget.rosbag_indicator,
            "#cc0000",
            on_success=self._print_warn,
        )

    def _arm(self):
        """
        Arm the selected vehicle.
        """
        self._set_indicator(
            self._current_agent, self._widget.armed_indicator, "#00cc00"
        )

    def _disarm(self):
        """
        Disarm the selected vehicle.
        """
        self._set_indicator(
            self._current_agent, self._widget.armed_indicator, "#cc0000"
        )

    def _acoustics_on(self):
        """
        Enable acoustics on the selected vehicle.
        """
        self._set_indicator(
            self._current_agent, self._widget.acoustics_indicator, "#00cc00"
        )

    def _acoustics_off(self):
        """
        Disable acoustics on the selected vehicle.
        """
        self._set_indicator(
            self._current_agent, self._widget.acoustics_indicator, "#cc0000"
        )

    def _emergency_stop(self):
        """
        Trigger emergency stop.
        """
        pass

    def _emergency_surface(self):
        """
        Trigger emergency surface.
        """
        pass

    def shutdown_plugin(self):
        """
        Clean up resources and destroy ROS clients when the plugin is shut down.
        """
        for ns_clients in self._clients.values():
            for client in ns_clients.values():
                self._node.destroy_client(client)
        self._clients = {}

    def save_settings(self, _plugin_settings, _instance_settings):
        """
        Save plugin settings.

        :param _plugin_settings: Plugin settings profile.
        :param _instance_settings: Instance settings profile.
        """
        pass

    def restore_settings(self, _plugin_settings, _instance_settings):
        """
        Restore plugin settings.

        :param _plugin_settings: Plugin settings profile.
        :param _instance_settings: Instance settings profile.
        """
        pass
