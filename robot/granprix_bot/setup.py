import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'granprix_bot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frayderMM',
    maintainer_email='fraydermezamorveli@gmail.com',
    description=(
        'Exploracion (flood fill) y speed run (BFS) para el laberinto '
        'Gran Prix CapyTown -- Yahboom ROSMASTER R2, ROS 2 Humble. '
        'Portado de simulador_pista.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer_node = granprix_bot.explorer_node:main',
            'speedrun_node = granprix_bot.speedrun_node:main',
        ],
    },
)
