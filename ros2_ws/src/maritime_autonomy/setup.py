import os
from glob import glob

from setuptools import setup

package_name = 'maritime_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'drone'), [f for f in glob('models/drone/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models', 'drone', 'meshes'), glob('models/drone/meshes/*')),
        (os.path.join('share', package_name, 'models', 'drone', 'materials'), [f for f in glob('models/drone/materials/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models', 'drone', 'materials', 'textures'), glob('models/drone/materials/textures/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mhpromit7473',
    maintainer_email='merajhossainpromit@gmail.com',
    description='Maritime surveillance autonomy skeleton (takeoff -> patrol -> RTL) for PX4 via MAVROS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'patrol_node = maritime_autonomy.patrol_node:main',
            'plan_node = maritime_autonomy.plan_node:main',
            'descend_node = maritime_autonomy.descend_node:main',
            'drop_node = maritime_autonomy.drop_node:main',
            'imu_sim_bridge = maritime_autonomy.imu_sim_bridge:main',
        ],
    },
)
