import os
from glob import glob

from setuptools import find_packages, setup


package_name = "pupil_labs_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamed Eyad",
    maintainer_email="mooeyad@gmail.com",
    description="ROS 2 driver for Pupil Labs Neon glasses",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pupil_labs_neon = pupil_labs_ros.pupil_labs_neon_ros:main",
        ],
    },
)
