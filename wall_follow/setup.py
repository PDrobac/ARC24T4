import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wall_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='T4',
    maintainer_email='T4@T4.com',
    description='Wall following mechanism',
    license='nothing',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follow = wall_follow.wall_follow:main'
        ],
    },
)
