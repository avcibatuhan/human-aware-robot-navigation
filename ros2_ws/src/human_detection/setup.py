from setuptools import find_packages, setup

package_name = "human_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Batuhan Avcı",
    maintainer_email="avcibatuhan26@gmail.com",
    description="YOLOv8 person detection on the robot's RGB camera.",
    license="AGPL-3.0-only",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": ["yolo_node = human_detection.yolo_node:main"]},
)
