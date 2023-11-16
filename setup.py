import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ground_station'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='polaris',
    maintainer_email='1223089722@qq.com',
    description='Ground Station',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radio_telemetry = ground_station.radio_telemetry:main',
            'visualizer = ground_station.visualizer:main'
        ],
    },
)
