import os
from glob import glob

from setuptools import setup

package_name = "ekf_localization"

setup(
    name=package_name,
    version="2.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "map.yaml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config", "ekf_localization_node"),
         glob("config/ekf_localization_node/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Liam P",
    maintainer_email="liam@duckietown.com",
    description="EKF localization of the Duckiebot from wheel encoders and AprilTag landmarks (ROS 2).",
    license="GPLv3",
    entry_points={
        "console_scripts": [
            "ekf_localization_node = ekf_localization.ekf_localization_node:main",
        ],
    },
)
