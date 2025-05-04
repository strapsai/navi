from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'spot_mobility_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,"launch"), glob("launch/*.*")),
        (os.path.join("share", package_name,"config"), glob("config/*.*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kabir Kedia',
    maintainer_email='kabirkedia0111@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "walk_forward = spot_mobility_controller.examples.walk_forward:main",
            "walk_forward_old = spot_mobility_controller.examples.walk_forward_old:main",
            "spot_controller = spot_mobility_controller.spot_controller:main",
            "odom_to_spot_tf_publisher = spot_mobility_controller.odom_to_spot_tf_publisher:main",
            "odom_controller = spot_mobility_controller.odom_controller:main",
        ],
    },
)
