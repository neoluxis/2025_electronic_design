import subprocess
import sys
import importlib
from typing import List, Tuple
import logging


def deps_check(
    packages: str | List[str] | Tuple[str] | None = None, 
    requirements: str | None = None,
    install: bool = True
):
    """Check whether the specified packages are installed, and optionally install them if not.

    Args:
        packages (str | List[str] | Tuple[str] | None): Package names to check.
        requirements (str | None): Path to a requirements file to check.
        If provided, this will override the `packages` argument.
        install (bool, optional): True to install if not installed. Defaults to True.
    """
    if isinstance(packages, str):
        packages = [packages]
    if packages is None and requirements is None:
        logging.warning("未指定任何包或要求文件。")
        return
    if requirements:
        try:
            with open(requirements, 'r') as f:
                packages = [line.strip() for line in f if line.strip() and not line.startswith('#')]
        except FileNotFoundError:
            logging.error(f"要求文件未找到: {requirements}")
            return
    instlist = []
    for pkg in packages:
        try:
            importlib.import_module(pkg)
            logging.info(f"[✔] 已安装: {pkg}")
        except ImportError:
            logging.warning(f"[✘] 未安装: {pkg}")
            instlist.append(pkg)
    if instlist and install:
        logging.info(f"正在安装: {' '.join(instlist)}")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install"] + instlist)
            logging.info("安装完成")
        except subprocess.CalledProcessError as e:
            logging.error(f"安装失败: {e}")


if __name__ == "__main__":
    deps_check(["numpy"], install=True)
