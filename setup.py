from setuptools import setup
import os
from glob import glob

package_name = "mcity_proxy"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.launch.py")),
        ("share/" + package_name + "/meshes", glob("meshes/*.stl")),
        ("share/" + package_name + "/models", glob("models/*.urdf")),
        ("share/" + package_name + "/models", glob("models/*.urdf.xacro")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),
        ("share/" + package_name + "/params", glob("params/*.yaml")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryan D. Lewis",
    maintainer_email="rdlrobot@umich.edu",
    description="ROS2 package providing all of the core functionality for Mcity's autonomous proxy.",
    license="MIT License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["mcity_proxy = mcity_proxy.mcity_proxy:main"],
    },
)
