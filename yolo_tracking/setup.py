from glob import glob

from setuptools import find_packages, setup


package_name = "yolo_tracking"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    package_data={"yolo_tracking.trackers": ["NOTICE", "LICENSE"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jalen Lv",
    maintainer_email="jalenlv543@qq.com",
    description="CPU multi-object tracking for YOLO detections",
    license="MIT",
    tests_require=["pytest"],
)
