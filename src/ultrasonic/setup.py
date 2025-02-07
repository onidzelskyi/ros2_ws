"""
Setup script for the ultrasonic_sensor package.

This script uses setuptools to package the ultrasonic_sensor ROS 2 package.
"""

from setuptools import find_packages, setup

PACKAGE_NAME = 'ultrasonic'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oleksii Nidzelskyi',
    maintainer_email='alexey.education@gmail.com',
    description='Ultrasonic HC-SR04 distance sensor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_publisher = ultrasonic.publisher:main',
            'dummy = ultrasonic.dummy_range:main',
        ],
    },
)
