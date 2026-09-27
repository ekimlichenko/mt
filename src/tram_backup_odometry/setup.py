"""ament_python build of tram_backup_odometry (maps, config and launch go to share/)."""
from glob import glob

from setuptools import find_packages, setup

package_name = 'tram_backup_odometry'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/maps', glob('maps/*.json')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Egor Kimlichenko',
    maintainer_email='egorkimlichenko@gmail.com',
    description='Backup odometry of a tram (wheel speeds + controller notch + track map; GNSS-free speed).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'backup_odometry_node = tram_backup_odometry.node:main',
            'replay = tram_backup_odometry.replay:main',
        ],
    },
)
