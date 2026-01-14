import warnings
# Suppress harmless setuptools warnings about pytest-repeat
warnings.filterwarnings('ignore', category=UserWarning, module='setuptools.command.easy_install')

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'alc_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alton',
    maintainer_email='59618908+kessellyalton@users.noreply.github.com',
    description='Gazebo simulation setup, world files, and ROS-Gazebo bridge configuration',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
