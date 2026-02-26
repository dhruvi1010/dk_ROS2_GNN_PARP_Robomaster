from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'ros_robomaster_description'

# print(glob('urdf/*'))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('urdf/config/*')),
        (os.path.join('share', package_name, 'urdf', 'functions'), glob('urdf/functions/*')),
        (os.path.join('share', package_name, 'urdf', 'gazebo_plugins'), glob('urdf/gazebo_plugins/*')),
        (os.path.join('share', package_name, 'urdf', 'parts'), glob('urdf/parts/*')),
        (os.path.join('share', package_name, 'urdf', 'meshes', 'base'), glob('urdf/meshes/base/*')),
        (os.path.join('share', package_name, 'urdf', 'meshes', 'lidar'), glob('urdf/meshes/lidar/*')),
        (os.path.join('share', package_name, 'urdf', 'meshes', 'wheels'), glob('urdf/meshes/wheels/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/robomaster.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niklas',
    maintainer_email='niklas.dietz@iml.fraunhofer.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'joint_tester = ros_robomaster_description.joint_tester:main'
        ],
    },
)
