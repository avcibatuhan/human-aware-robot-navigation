import os
from glob import glob

from setuptools import find_packages, setup

package_name = "human_bringup"
share = os.path.join("share", package_name)


def tree(directory):
    """Install every file under ``directory``, keeping the folder structure."""
    return [
        (os.path.join(share, root), [os.path.join(root, f) for f in files])
        for root, _, files in os.walk(directory)
        if files
    ]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (share, ["package.xml"]),
        (os.path.join(share, "launch"), glob("launch/*.launch.py")),
        (os.path.join(share, "config"), glob("config/*")),
        (os.path.join(share, "rviz"), glob("rviz/*")),
        (os.path.join(share, "urdf"), glob("urdf/*")),
        (os.path.join(share, "maps"), glob("maps/*")),
        *tree("worlds"),
        *tree("models"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Batuhan Avcı",
    maintainer_email="avcibatuhan26@gmail.com",
    description="Simulation world, robot model, launch files and parameters "
    "for human-aware navigation.",
    license="AGPL-3.0-only",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
