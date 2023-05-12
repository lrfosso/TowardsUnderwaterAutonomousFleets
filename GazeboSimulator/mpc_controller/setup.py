"""
_________________________________________________________________________________________________________
Setting up the package so ROS 2 knows how it should be installed
_________________________________________________________________________________________________________
Resource:
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
_________________________________________________________________________________________________________
"""

# Import modules
import os
from glob import glob
from setuptools import setup

package_name = 'mpc_controller'

# Setting up package
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='E2310',
    maintainer_email='',
    description='Bachelor thesis - https://github.com/lrfosso/TowardsUnderwaterAutonomousFleets',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={ # Gives executable names to nodes
        'console_scripts': [
        'GUI = mpc_controller.GUI:main',
        'bluerov_mpc = mpc_controller.mpc_controller:main',
        'setpoint = mpc_controller.setpoint_publisher:main',
        ],
    },
)
