from setuptools import find_packages, setup

package_name = 'comms_monitor_pynode'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhruvi',
    maintainer_email='dhruvikoshiya1010@gmail.com',
    description='Periodic 5G / Wi-Fi link-quality probe; publishes perception_aware_nav2_msgs/LinkStats.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    
    entry_points={
    'console_scripts': [
        'comms_monitor_pynode = comms_monitor_pynode.comms_monitor_pynode:main',
    ],
},
)
