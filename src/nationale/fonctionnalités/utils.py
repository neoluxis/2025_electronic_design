import os

from PySide6.QtWidgets import QApplication, QMessageBox, QPushButton, QWidget, QVBoxLayout

def confirm_action(window, msg="Are you sure you want to proceed?"):
    reply = QMessageBox.question(
        window,
        "确认操作",
        msg,
        QMessageBox.Yes | QMessageBox.No
    )

    if reply == QMessageBox.Yes:
        return True
    else:
        return False

def get_debug_mode():
    return os.getenv("DEBUG", "False").lower() in ("true", "1", "yes")