from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'nationale'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/assets', glob('assets/*')),
        ('share/' + package_name + '/configs', glob('configs/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neolux',
    maintainer_email='neolux_lee@outlook.com',
    description='2025 nationale skeleton',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "app_cu_dl = nationale.app_cu_dl:main",
            "app_cuno_dl = nationale.app_cuno_dl:main",
            "threshold_filter = apps.threshold_filter:main",
        ],
    },
)
