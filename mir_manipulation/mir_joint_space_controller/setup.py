#!/usr/bin/env python
"""
//TODO
"""
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

DIST_UTILS_OBJ = generate_distutils_setup(
    packages=["mir_joint_space_controller_ros"],
    package_dir={"mir_joint_space_controller_ros": "ros/src/mir_joint_space_controller_ros"},
)

setup(**DIST_UTILS_OBJ)
