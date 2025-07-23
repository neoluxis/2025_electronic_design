#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @author: Neolux Lee
# @email: neolux_lee@outlook.com
# @date: 2026-06-29
# @description: Experiment manager for creating unique experiment directories

import os
import time
import datetime


def mkexpdir(project="runs", prefix="", withdate=False):
    """自动创建 {project}/{prefix}X 文件夹，X 为递增数字"""
    os.makedirs(f"{project}", exist_ok=True)
    i = 0
    if withdate:
        date_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix = f"{prefix}_{date_str}_"
    while True:
        run_folder = f"{project}/{prefix}{i}"
        if not os.path.exists(run_folder):
            os.makedirs(run_folder)
            return run_folder
        i += 1
    return None


if __name__ == "__main__":
    # Test the function
    run_folder = mkexpdir(project="runs", prefix="test", withdate=True)
    print(f"Created run folder: {run_folder}")
    time.sleep(1)  # Wait for a second to see the difference in folder names
    run_folder2 = mkexpdir(project="runs", prefix="test", withdate=True)
    print(f"Created another run folder: {run_folder2}")
    # This should create two folders with different timestamps
    run_folder3 = mkexpdir(project="runs", prefix="test")
    print(f"Created run folder without date: {run_folder3}")
