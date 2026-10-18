import os
from glob import glob

from setuptools import setup

package_name = 'tracker_sim'


def tree(src):
    """data_files entries for every file below src, preserving the layout."""
    out = {}
    for path in glob(os.path.join(src, '**', '*'), recursive=True):
        if os.path.isfile(path):
            out.setdefault(os.path.join('share', package_name, os.path.dirname(path)), []).append(path)
    return list(out.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Meraj Hossain Promit',
    maintainer_email='merajhossainpromit@gmail.com',
    description='Gazebo city world, drone model, camera bridge and traffic.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gz_camera_bridge = tracker_sim.gz_camera_bridge:main',
            'traffic_node = tracker_sim.traffic_node:main',
        ],
    },
)
