import os
from glob import glob

from setuptools import setup

package_name = "hunavis"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.py")),
        (os.path.join("share", package_name, "params"), glob("params/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michael Lu",
    maintainer_email="michael_lu_3@sfu.ca",
    description="Utilities for HUNAVsim, NAV2, and VISualization",
    license="",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "people_visualizer = hunavis.people_visualizer:main",
        ],
    },
)
