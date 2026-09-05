from glob import glob
import os

from setuptools import setup

package_name = 'tactile_umi'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tactile-UMI maintainers',
    maintainer_email='maintainer@example.com',
    description='Handheld visuo-tactile data collection: publisher, sync monitor, MCAP recorder.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visuo_tactile_publisher = tactile_umi.publisher_node:main',
            'sync_monitor = tactile_umi.sync_monitor_node:main',
            'mcap_recorder = tactile_umi.mcap_recorder_node:main',
            'firmware_emulator = tactile_umi.firmware_emulator:main',
        ],
    },
)
