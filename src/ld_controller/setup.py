from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ld_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lance Downton',
    maintainer_email='snyvie7@gmail.com',
    description='Take a controller input and translate it to mouse mouvement',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = ld_controller.controller_publisher:main',
            'subscriber = ld_controller.controller_subscriber:main'
        ],
    },
)
