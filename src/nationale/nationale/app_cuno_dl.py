import os
import datetime
import time
from functools import partial
import commentjson

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CompressedImage

import sys
from PySide6 import QtCore, QtGui
from PySide6.QtCore import Qt, QFile
from PySide6 import QtWidgets
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QGraphicsScene,
    QGraphicsPixmapItem,
)
from PySide6.QtGui import QPixmap, QImage
import cv2

import nlxpy
from nlxpy.ros2.imger import cv2ros, ros2cv

from fonctionnalités import systematique
from fonctionnalités.utils import confirm_action, get_debug_mode
from nationale_interfaces.srv import ThresholdSetter

debug = get_debug_mode()


class AppWindow(QMainWindow):
    def __init__(self, node: Node = None):
        super(AppWindow, self).__init__()

        self.assets_path = os.path.join(
            get_package_share_directory("nationale"), "assets"
        )
        configs_path = os.path.join(get_package_share_directory("nationale"), "configs")
        self.configs_path = "./configs"
        if not os.path.exists(self.configs_path):
            os.system(f"cp -rv {configs_path} {self.configs_path}")

        self.load_ui()

        self.scene = QGraphicsScene()
        self.scene_proc = QGraphicsScene()

        self.ui.imageOrig.setScene(self.scene)
        self.ui.imageOrig.setRenderHint(QtGui.QPainter.Antialiasing)
        self.ui.imageProc.setScene(self.scene_proc)
        self.ui.imageProc.setRenderHint(QtGui.QPainter.Antialiasing)

        self.node = node
        if self.node:
            self.node.get_logger().info("Node is set in AppWindow.")
        else:
            self.node.get_logger().warn("Node is not set in AppWindow.")

        self.threshold_path = os.path.join(self.configs_path, "thresholds.jsonc")
        self.thres_sliders = [
            self.ui.sldColorMode,
            self.ui.sldv1,
            self.ui.sldv2,
            self.ui.sldv3,
            self.ui.sldv4,
            self.ui.sldv5,
            self.ui.sldv6,
        ]
        self.thres_labels = [
            self.ui.lbvColorMode,
            self.ui.lbv1,
            self.ui.lbv2,
            self.ui.lbv3,
            self.ui.lbv4,
            self.ui.lbv5,
            self.ui.lbv6,
        ]
        self.thres = self.get_thres_from_json()
        self.on_threshold_slider_released()

        self.bind_slots()

        self.logging(
            "info",
            f"AppWindow initialized. With debug mode: {'on' if debug else 'off'}",
        )

    def bind_slots(self):
        self.ui.btnShutdown.clicked.connect(partial(systematique.shutdown, self))
        self.ui.btnReboot.clicked.connect(partial(systematique.reboot, self))
        for i, slider in enumerate(self.thres_sliders):
            slider.valueChanged.connect(
                lambda value, index=i: self.on_threshold_changed(value, index)
            )
            slider.sliderReleased.connect(
                lambda index=i: self.on_threshold_slider_released(index)
            )
        # self.ui.sldThres.sliderReleased.connect(self.on_threshold_slider_released)

    def load_ui(self):
        self.uipath = os.path.join(self.assets_path, "app_traditional.ui")
        self.loader = QUiLoader()
        ui_file = QFile(self.uipath)
        ui_file.open(QFile.ReadOnly)
        self.ui = self.loader.load(ui_file, self)
        ui_file.close()

        self.setWindowTitle("2025 电赛国赛のアプリ")
        self.showMaximized()

    def get_thres_from_json(self):
        """
        Get threshold value from the model.json file.
        """
        with open(self.threshold_path, "r") as f:
            model_config = commentjson.load(f)
            self.color_modes_candidate = model_config.get(
                "color_modes_candidate", ["bgr", "hsv"]
            )
            self.color_mode = model_config.get("color_mode", "bgr")
            color_mode_index = self.color_modes_candidate.index(self.color_mode)
            self.ui.sldColorMode.setValue(color_mode_index)
            self.ui.lbvColorMode.setText(self.color_mode)
            self.thresholds = model_config.get("thresholds", [0] * 6)
            for i, thres in enumerate(self.thresholds):
                self.thres_sliders[i + 1].setValue(thres)
                self.thres_labels[i + 1].setText(f"{thres:3d}")

    def on_threshold_changed(self, value, index):
        if index == 0:
            self.color_mode = self.color_modes_candidate[value]
            self.ui.lbvColorMode.setText(self.color_mode)
            self.logging("info", f"Color mode set to {self.color_mode}")
        else:
            thres_value = value
            self.thres_labels[index].setText(f"{thres_value:3d}")
            self.logging("info", f"Threshold {index} set to {thres_value}")
        self.node.set_thresholds(
            self.color_mode,
            [slider.value() for slider in self.thres_sliders[1:]],
        )

    def on_threshold_slider_released(self, index=None):
        if index is None:
            model_config = {
                "color_mode": self.color_mode,
                "thresholds": [slider.value() for slider in self.thres_sliders[1:]],
            }
        elif index == 0:
            self.color_mode = self.color_modes_candidate[self.ui.sldColorMode.value()]
            self.ui.lbvColorMode.setText(self.color_mode)
            self.logging("info", f"Color mode set to {self.color_mode}")
            with open(self.threshold_path, "r") as f:
                model_config = commentjson.load(f)
            model_config["color_mode"] = self.color_mode
            self.logging(
                "info",
                f"Color mode updated in {self.threshold_path}: {self.color_mode}",
            )
            if self.color_mode == "hsv":
                self.ui.sldv1.setMaximum(179)
                self.ui.sldv2.setMaximum(179)
                self.ui.sldv3.setMaximum(255)
                self.ui.sldv4.setMaximum(255)
                self.ui.sldv5.setMaximum(255)
                self.ui.sldv6.setMaximum(255)
                self.logging("info", "Switched to HSV color mode.")
            elif self.color_mode == "bgr":
                self.ui.sldv1.setMaximum(255)
                self.ui.sldv2.setMaximum(255)
                self.ui.sldv3.setMaximum(255)
                self.ui.sldv4.setMaximum(255)
                self.ui.sldv5.setMaximum(255)
                self.ui.sldv6.setMaximum(255)
                self.logging("info", "Switched to BGR color mode.")
            
        else:
            thres_value = self.thres_sliders[index].value()
            self.thres_labels[index].setText(f"{thres_value:3d}")
            self.logging("info", f"Threshold {index} set to {thres_value}")
            with open(self.threshold_path, "r") as f:
                model_config = commentjson.load(f)
            model_config["thresholds"][index - 1] = thres_value
            self.logging(
                "info",
                f"Thresholds updated in {self.threshold_path}: {model_config['thresholds']}",
            )
        with open(self.threshold_path, "w") as f:
            commentjson.dump(model_config, f, indent=4)

    def logging(self, level, msg):
        """
        Log messages to the UI text area.
        """
        if level == "info":
            self.ui.textLogging.appendPlainText(
                f"[{datetime.datetime.now()}] INFO: {msg}"
            )
        elif level == "error":
            self.ui.textLogging.appendPlainText(
                f"[{datetime.datetime.now()}] ERROR: {msg}"
            )
        elif level == "debug":
            self.ui.textLogging.appendPlainText(
                f"[{datetime.datetime.now()}] DEBUG: {msg}"
            )
        else:
            self.ui.textLogging.appendPlainText(
                f"[{datetime.datetime.now()}] LOG: {msg}"
            )


class GuiNode(Node):
    def __init__(self):
        super().__init__("app_with_dl")
        self.get_logger().info("app_with_dl init.")
        self.window = None
        self.image_orig_topic = (
            self.declare_parameter("image_topic", "/image_mjpeg")
            .get_parameter_value()
            .string_value
        )
        self.image_proc_topic = (
            self.declare_parameter("image_proc_topic", "/threshold_filter/mask")
            .get_parameter_value()
            .string_value
        )

        self.image_sub = self.create_subscription(
            CompressedImage, self.image_orig_topic, self.image_callback, 10
        )
        self.image_proc_sub = self.create_subscription(
            CompressedImage, self.image_proc_topic, self.image_proc_callback, 10
        )

        # 图像项初始化为 None
        self.pixmap_item = None
        self.pixmap_proc_item = None
        self.frame_count = 0

        self.status_timer = self.create_timer(1.0, self.status_update_callback)

        self.threshold_client = self.create_client(
            ThresholdSetter, "/threshold_filter/set_thresholds"
        )

    def set_thresholds(self, color_mode, thresholds):
        if not self.threshold_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("ThresholdSetter not available.")
            return False

        request = ThresholdSetter.Request()
        request.color_mode = color_mode
        request.thresholds = thresholds

        future = self.threshold_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Thresholds set successfully.")
            return True
        else:
            self.get_logger().error("Failed to set thresholds.")
            return False

    def image_callback(self, msg: CompressedImage):
        img = ros2cv(msg)
        if img is None:
            return

        # OpenCV 图像通常是 BGR，需要转为 RGB
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # 构造 QImage
        height, width, channel = img_rgb.shape
        bytes_per_line = 3 * width
        qimg = QImage(img_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)

        pixmap = QPixmap.fromImage(qimg)

        if self.pixmap_item is None:
            self.pixmap_item = QGraphicsPixmapItem(pixmap)
            self.window.scene.addItem(self.pixmap_item)
        else:
            self.pixmap_item.setPixmap(pixmap)

        # 保持图像适应视图
        self.window.ui.imageOrig.fitInView(self.pixmap_item, Qt.KeepAspectRatio)
        self.frame_count += 1
        
    def image_proc_callback(self, msg: CompressedImage):
        img = ros2cv(msg)
        if img is None:
            return

        # OpenCV 图像通常是 BGR，需要转为 RGB
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # 构造 QImage
        height, width, channel = img_rgb.shape
        bytes_per_line = 3 * width
        qimg = QImage(img_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)

        pixmap = QPixmap.fromImage(qimg)

        if self.pixmap_proc_item is None:
            self.pixmap_proc_item = QGraphicsPixmapItem(pixmap)
            self.window.scene_proc.addItem(self.pixmap_proc_item)
        else:
            self.pixmap_proc_item.setPixmap(pixmap)

        # 保持图像适应视图
        self.window.ui.imageProc.fitInView(self.pixmap_proc_item, Qt.KeepAspectRatio)

    def status_update_callback(self):
        self.get_logger().info(f"FPS: {self.frame_count}")
        # self.window.logging("info", f"FPS: {self.frame_count}")
        self.window.ui.lblFPS.setText(f"FPS: {self.frame_count}")
        self.frame_count = 0


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = GuiNode()
    window = AppWindow(node)
    node.window = window
    window.show()

    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    timer.start(5)

    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
