from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vn_keyboard_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Include launch files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # Include config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toaster',
    maintainer_email='toaster@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "keyboard_controller = vn_keyboard_controller.keyboard_controller:main",
            "twist_node = vn_keyboard_controller.twist_node:main"
        ],
    },
)
