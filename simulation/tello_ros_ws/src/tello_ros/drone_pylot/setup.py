from setuptools import find_packages, setup

package_name = "drone_pylot"

setup(
    name=package_name,
    version="0.0.0",
    packages=['drone_pylot'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=["setuptools", "tello_msgs"],
    zip_safe=True,
    maintainer="olivia",
    maintainer_email="osmi3043@uni.sydney.edu.au",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'fsm = drone_pylot.fsm:main',
            'aruco_tracker = drone_pylot.aruco_trajectory_estimator:main',
        ],
    },
)
