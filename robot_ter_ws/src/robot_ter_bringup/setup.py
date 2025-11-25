from glob import glob
from setuptools import setup

package_name = "robot_ter_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mostafa",
    maintainer_email="mostafa@example.com",
    description="Custom Gazebo world & TurtleBot3 simulation bringup for Task 1.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={},
)
