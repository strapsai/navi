from setuptools import setup

package_name = 'spot_isaac_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/spot.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adi',
    maintainer_email='rauniyar@cmu.edu',
    description='Adapter between Boston Dynamics Spot and Isaac Sim using ROS 2.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boston_dynamics2ros = spot_isaac_adapter.boston_dynamics2ros:main',
            'ros2isaac_policy = spot_isaac_adapter.ros2isaac_policy:main',
        ],
    },
)
