from glob import glob
from setuptools import find_packages, setup


package_name = "b2arx_nav2_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lbz",
    maintainer_email="lbz@example.com",
    description="B2ARX ZED/Nvblox/Nav2 simulation bringup",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "behavior_footprint_publisher = b2arx_nav2_bringup.behavior_footprint_publisher:main",
            "cmd_vel_watchdog = b2arx_nav2_bringup.cmd_vel_watchdog:main",
        ],
    },
)
