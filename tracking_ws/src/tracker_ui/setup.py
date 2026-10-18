from setuptools import setup

package_name = 'tracker_ui'

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
    description='Operator console for the drone tracker.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'operator_console = tracker_ui.operator_console:main',
        ],
    },
)
