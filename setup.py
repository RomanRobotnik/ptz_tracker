import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ptz_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*yaml'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rnavarro',
    maintainer_email='rnavarro@robotnik.es',
    description='PTZ tracker action server',
    license='BSD',
    entry_points={
        'console_scripts': [
            'ptz_tracker_node = ptz_tracker.ptz_tracker_node:main',
        ],
    },
)
