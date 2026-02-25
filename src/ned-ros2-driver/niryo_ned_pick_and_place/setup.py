from setuptools import find_packages, setup

package_name = "niryo_ned_pick_and_place"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/pick_and_place.yaml"]),
    ],
    install_requires=["setuptools"],
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
            "pick_and_place_node = niryo_ned_pick_and_place.pick_and_place_node:main"
        ],
    },
)
