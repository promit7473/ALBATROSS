from setuptools import setup

package_name = 'tracker_hw'

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
    description='Real drone I/O: RTSP camera and the Intelis telemetry bridge link.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtsp_camera = tracker_hw.rtsp_camera:main',
            'bridge_link = tracker_hw.bridge_link:main',
        ],
    },
)
