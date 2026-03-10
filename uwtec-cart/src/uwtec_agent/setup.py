from setuptools import find_packages, setup
import os
from glob import glob


package_name = "uwtec_agent"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "script"), glob("script/*.py")),
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
            "demo_node = uwtec_agent.demo_node:main",
            "uwtec_agent = uwtec_agent.agent_afr:main",
            # "uagent = uwtec_agent.agent_afr:main",
        ],
    },
)
