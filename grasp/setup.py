from setuptools import find_packages, setup

package_name = "rim_grasp_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/left.yaml",
            "config/left_ik.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/left_wrist.launch.py",
            "launch/left_ik_only.launch.py",
        ]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "rim_grasp_node = rim_grasp_perception.ros_node:main",
            "rim_grasp_offline = rim_grasp_perception.offline:main",
            "left_ik_client = rim_grasp_perception.ik_client:main",
        ]
    },
)
