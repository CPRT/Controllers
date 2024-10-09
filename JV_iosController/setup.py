from setuptools import find_packages, setup

package_name = 'JV_iosController'

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
    maintainer='Jimmy Vaughan',
    maintainer_email='jimmyvaughan@cmail.carleton.ca',
    description='Dug driver described as driving due to drab iOS app.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twister = JV_iosController.twister:main'
        ],
    },
)
