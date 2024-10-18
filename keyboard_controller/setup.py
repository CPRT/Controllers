from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'keyboard_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('robot_build', 'launch'), glob('launch/*.launch.py')),
        (os.path.join('robot_build','config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jayden',
    maintainer_email='chengjayden442@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = keyboard_controller.controller:main',
            'twistPub = keyboard_controller.twistPub:main'
        ],
    },
)
