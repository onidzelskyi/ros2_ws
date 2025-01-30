from setuptools import find_packages, setup

package_name = 'ultrasonic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
