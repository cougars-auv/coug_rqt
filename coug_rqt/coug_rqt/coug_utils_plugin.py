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
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
        for ns in self._node.get_parameter("agent_namespaces").value:
            if ns:
                self._widget.agent_selector.addItem(ns)

        self._bag_record_client = None
        self._current_agent = ""
        self._widget.agent_selector.currentTextChanged.connect(self._on_agent_changed)
        self._widget.rosbag_start.clicked.connect(self._rosbag_start)
        self._widget.rosbag_stop.clicked.connect(self._rosbag_stop)
        self._widget.arm.clicked.connect(self._arm)
        self._widget.disarm.clicked.connect(self._disarm)
        self._widget.acoustics_on.clicked.connect(self._acoustics_on)
        self._widget.acoustics_off.clicked.connect(self._acoustics_off)
        self._widget.emergency_stop.clicked.connect(self._emergency_stop)
        self._widget.emergency_surface.clicked.connect(self._emergency_surface)

    def _on_agent_changed(self, text):
        self._current_agent = text
        self._bag_record_client = self._node.create_client(
            BagRecord, f"{text}/bag_record"
        )

    def _call_service(self, client, request, indicator, color):
        if client is None:
            return
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: QTimer.singleShot(
                0, lambda: self._on_service_done(f, indicator, color)
            )
        )

    def _on_service_done(self, future, indicator, color):
        if future.result() is not None and future.result().success:
            indicator.setStyleSheet(f"background-color: {color}; border-radius: 6px;")

    def _rosbag_start(self):
        req = BagRecord.Request()
        req.start = True
        req.prefix = self._widget.bag_prefix.text()
        self._call_service(
            self._bag_record_client, req, self._widget.rosbag_indicator, "#00cc00"
        )

    def _rosbag_stop(self):
        req = BagRecord.Request()
        req.start = False
        req.prefix = ""
        self._call_service(
            self._bag_record_client, req, self._widget.rosbag_indicator, "#cc0000"
        )

    def _arm(self):
        self._widget.armed_indicator.setStyleSheet(
            "background-color: #00cc00; border-radius: 6px;"
        )

    def _disarm(self):
        self._widget.armed_indicator.setStyleSheet(
            "background-color: #cc0000; border-radius: 6px;"
        )

    def _acoustics_on(self):
        self._widget.acoustics_indicator.setStyleSheet(
            "background-color: #00cc00; border-radius: 6px;"
        )

    def _acoustics_off(self):
        self._widget.acoustics_indicator.setStyleSheet(
            "background-color: #cc0000; border-radius: 6px;"
        )

    def _emergency_stop(self):
        pass

    def _emergency_surface(self):
        pass

    def shutdown_plugin(self):
        pass

    def save_settings(self, _plugin_settings, _instance_settings):
        pass

    def restore_settings(self, _plugin_settings, _instance_settings):
        pass
