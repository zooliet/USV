from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'uwtec_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junhyun Shin',
    maintainer_email='hl1sqi@gmail.com',
    description='TODO: Package description',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'demo_node = uwtec_navigation.demo_node:main',
            "cmd_vel_joy = uwtec_navigation.cmd_vel_joy:main",
            # "heading_and_offset = uwtec_navigation.heading_and_offset_server:main",
            # "shuttle_run = uwtec_navigation.shuttle_run_server:main",
            # "nav_to_wps = uwtec_navigation.nav_to_wps_server:main",
            "nav_server = uwtec_navigation.nav_server:main",
            # "action_server_demo = uwtec_navigation.action_server_demo:main",
        ],
    },
)
