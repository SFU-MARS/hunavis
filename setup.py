import os
from glob import glob

from setuptools import setup

package_name = "hunavis"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "maps"), glob("maps/*")),
    (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
    (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    (os.path.join("share", package_name, "launch"), glob("launch/*")),
    (os.path.join("share", package_name, "params"), glob("params/*")),
]

for directory in glob("models/*"):
    files = glob(directory + "/*")
    data_files.append((os.path.join("share", package_name, directory), files))

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mo Chen",
    maintainer_email="mochen@cs.sfu.ca",
    description="Utilities for HUNAVsim, NAV2, and VISualization",
    license="",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "publish_global_pose = hunavis.robot_pose_publisher:main",            
            "people_visualizer = hunavis.people_visualizer:main",
        ],
    },
)
