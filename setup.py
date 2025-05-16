from setuptools import find_packages, setup

package_name = "drone-PYlot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="olivia",
    maintainer_email="osmi3043@uni.sydney.edu.au",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "node = drone-PYlot.cone_distance_detection_node:main",
        ],
    },
)
