from setuptools import find_packages, setup

package_name = "human_tracking"

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
    description="DeepSORT tracking, 3D localization and map-frame transform of detected humans.",
    license="AGPL-3.0-only",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "deepsort_node = human_tracking.deepsort_node:main",
            "human_localizer = human_tracking.human_localizer:main",
            "human_frame_transformer = human_tracking.human_frame_transformer:main",
        ]
    },
)
