from setuptools import setup

package_name = 'tracker_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Meraj Hossain Promit',
    maintainer_email='merajhossainpromit@gmail.com',
    description='Geolocation, target estimation, follow guidance and flight management.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_node = tracker_control.follow_node:main',
        ],
    },
)
