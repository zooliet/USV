from setuptools import find_packages, setup
import os
from glob import glob

package_name = "uwtec_cart"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "routes"), glob("routes/*.yaml")),
        (os.path.join("share", package_name, "script"), glob("script/*.py")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Junhyun Shin",
    maintainer_email="hl1sqi@gmail.com",
    description="TODO: Package description",
    license="BSD-3-Clause",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "demo_node = uwtec_cart.demo_node:main",
            "localizer = uwtec_cart.localizer:main",
            "navigator = uwtec_cart.navigator:main",
            "agent = uwtec_cart.agent:main",
            "cmd_vel_joy = uwtec_cart.cmd_vel_joy:main",
            "um982_config = uwtec_cart.um982_configurator:main",
            "streamer = uwtec_cart.ultrasonic_rtsp:main",
        ],
    },
)
