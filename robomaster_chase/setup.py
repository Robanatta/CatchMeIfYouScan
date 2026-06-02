from setuptools import setup
from glob import glob

package_name = "robomaster_chase"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.xml")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.rviz")),
        # Saved SLAM maps (room.posegraph/.data for slam_toolbox localization,
        # room.yaml/.pgm for viewing / future AMCL). See robot_a_localization_plan.
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Diell",
    maintainer_email="kryeziudiell@gmail.com",
    description="RoboMaster autonomous navigation: Nav2 + SLAM waypoint tour (sim).",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            # Nav2 solution: autonomous waypoint tour (nav2.launch.py).
            "nav2_waypoint_sender = robomaster_chase.nav2_waypoint_sender:main",
            "scan_restamp = robomaster_chase.scan_restamp:main",
            # Manual teleop, for the manual scan -> save -> Nav2 localization flow
            # (slam.launch.py + keyboard_controller).
            "keyboard_controller = robomaster_chase.keyboard_controller:main",
        ],
    },
)
