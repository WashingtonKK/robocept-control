from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robocept_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WashingtonKK',
    maintainer_email='washingtonkigan@gmail.com',
    description='Differential-drive base controller for robocept',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver = robocept_base.base_driver:main',
            'base_sim_adapter = robocept_base.base_sim_adapter:main',
        ],
    },
)
