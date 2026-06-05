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
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin


class CougUtilsPlugin(Plugin):
    """
    RQT panel for per-agent utility commands (arm/disarm and DVL controls).

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

        node = context.node
        try:
            node.declare_parameter("agent_namespaces", [""])
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
        for ns in node.get_parameter("agent_namespaces").value:
            if ns:
                self._widget.agent_selector.addItem(ns)

        self._current_agent = ""
        self._widget.agent_selector.currentTextChanged.connect(self._on_agent_changed)
        self._widget.arm.clicked.connect(self._arm)
        self._widget.disarm.clicked.connect(self._disarm)
        self._widget.dvl_on.clicked.connect(self._dvl_on)
        self._widget.dvl_off.clicked.connect(self._dvl_off)

    def _on_agent_changed(self, text):
        self._current_agent = text

    def _arm(self):
        self._widget.armed_indicator.setStyleSheet(
            "background-color: #00cc00; border-radius: 6px;"
        )

    def _disarm(self):
        self._widget.armed_indicator.setStyleSheet(
            "background-color: #cc0000; border-radius: 6px;"
        )

    def _dvl_on(self):
        self._widget.dvl_indicator.setStyleSheet(
            "background-color: #00cc00; border-radius: 6px;"
        )

    def _dvl_off(self):
        self._widget.dvl_indicator.setStyleSheet(
            "background-color: #cc0000; border-radius: 6px;"
        )

    def shutdown_plugin(self):
        pass

    def save_settings(self, _plugin_settings, _instance_settings):
        pass

    def restore_settings(self, _plugin_settings, _instance_settings):
        pass
