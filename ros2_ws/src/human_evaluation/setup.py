from setuptools import find_packages, setup

package_name = "human_evaluation"

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
    description="Live and offline evaluation of human tracking and human-aware navigation.",
    license="AGPL-3.0-only",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "live_summary = human_evaluation.live_summary:main",
            "run_goal = human_evaluation.run_goal:main",
            "bag_summary = human_evaluation.bag_summary:main",
            "compare_navigation = human_evaluation.compare_navigation:main",
            "export_navigation_results = human_evaluation.export_navigation_results:main",
        ]
    },
)
