import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'reactive'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
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
    description='Follow the gap mechanism',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "reactive = reactive.reactive:main",
            "reactive_front = reactive.reactive_front_race:main"
        ],
    },
)
