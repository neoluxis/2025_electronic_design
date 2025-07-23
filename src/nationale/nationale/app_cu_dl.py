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

        self.setWindowTitle("2025 电赛国赛のアプリ")
        self.showMaximized() 

        self.scene = QGraphicsScene()

        self.ui.imageOrig.setScene(self.scene)
        self.ui.imageOrig.setRenderHint(QtGui.QPainter.Antialiasing)

        self.node = node
        if self.node:
            self.node.get_logger().info("Node is set in AppWindow.")
        else:
            self.node.get_logger().warn("Node is not set in AppWindow.")

        self.model_path = os.path.join(self.configs_path, "model.jsonc")
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
        self.ui.sldThres.valueChanged.connect(self.on_threshold_changed)
        self.ui.sldThres.sliderReleased.connect(self.on_threshold_slider_released)

    def load_ui(self):
        self.uipath = os.path.join(self.assets_path, "app_with_deeplearning.ui")
        self.loader = QUiLoader()
        ui_file = QFile(self.uipath)
        ui_file.open(QFile.ReadOnly)
        self.ui = self.loader.load(ui_file, self)
        ui_file.close()

    def get_thres_from_json(self):
        """
        Get threshold value from the model.json file.
        """
        try:
            with open(self.model_path, "r") as f:
                model_config = commentjson.load(f)
                thres = model_config.get("score_threshold", 0.5)
                self.ui.lblThresValue.setText(f"{thres:.02f}")
                self.ui.sldThres.setValue(int(thres * 100))  # Convert to percentage
                return thres
        except FileNotFoundError:
            self.logging(
                "error",
                f"Model configuration file not found at {self.model_path}. Using default threshold 0.5.",
            )
            self.ui.lblThresValue.setText("0.50")
            self.ui.sldThres.setValue(50)
            return 0.5

    def on_threshold_changed(self, value):
        self.thres = value / 100.0
        self.ui.lblThresValue.setText(f"{self.thres:.02f}")

    def on_threshold_slider_released(self):
        self.logging("info", f"Threshold set to {self.thres:.02f}")
        with open(self.model_path, "r") as f:
            model_config = commentjson.load(f)
            model_config["score_threshold"] = self.thres
        with open(self.model_path, "w") as f:
            commentjson.dump(model_config, f, indent=4)
        self.logging(
            "info",
            f"Threshold updated in model.json: {self.model_path} to {self.thres:.02f}",
        )

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

        self.image_sub = self.create_subscription(
            CompressedImage, "/image_mjpeg", self.image_callback, 10
        )

        # 图像项初始化为 None
        self.pixmap_item = None
        self.frame_count = 0

        self.status_timer = self.create_timer(1.0, self.status_update_callback)

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
