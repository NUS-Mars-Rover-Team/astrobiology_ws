import os
from glob import glob

from setuptools import setup

package_name = 'raman_spectrometer'

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
    description='B&W Tek BTC100 Raman spectrometer node publishing Raman spectra',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'raman_node = raman_spectrometer.raman_node:main',
        ],
    },
)
