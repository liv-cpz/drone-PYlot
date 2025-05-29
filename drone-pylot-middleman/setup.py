import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'drone-pylot-middleman'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'resource/ost.txt', 'resource/ost.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olivia',
    maintainer_email='osmi3043@uni.sydney.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
