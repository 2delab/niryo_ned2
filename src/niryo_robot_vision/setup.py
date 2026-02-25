from setuptools import find_packages, setup

package_name = "niryo_robot_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "scipy>=1.10.0"],
    zip_safe=True,
    maintainer="i",
    maintainer_email="2dellab@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "camera_calibration = niryo_robot_vision.camera_calibration:main",
            "aruco_pose = niryo_robot_vision.arucopose:main",
            "distance_pose = niryo_robot_vision.distance:main",
            "cube_aruco = niryo_robot_vision.cubearuco:main",
        ],
    },
)
