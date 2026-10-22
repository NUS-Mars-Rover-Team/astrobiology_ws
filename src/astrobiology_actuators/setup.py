import os
from glob import glob

from setuptools import setup

package_name = 'astrobiology_actuators'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CS Seow',
    maintainer_email='chengxi03@gmail.com',
    description='Actuator nodes for the astrobiology payload (pumps, gantry, centrifuge, carousel)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pump_node = astrobiology_actuators.pump_node:main',
            'lights_node = astrobiology_actuators.lights_node:main',
        ],
    },
)
