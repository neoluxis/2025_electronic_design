import os
import sys

from .utils import confirm_action, get_debug_mode

debug = get_debug_mode()


def shutdown(window=None):
    """
    Shutdown system
    """
    if confirm_action(window, "Are you sure you want to shut down?"):
        if window:
            window.logging("info", "Shutting down system...")
        if not debug:
            os.system("sudo shutdown now")


def reboot(window=None):
    """
    Reboot system
    """
    if confirm_action(window, "Are you sure you want to reboot?"):
        if window:
            window.logging("info", "Rebooting system...")
        if not debug:
            os.system("sudo reboot")
