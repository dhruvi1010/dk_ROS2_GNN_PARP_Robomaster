from setuptools import setup

package_name = 'comms_modem_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Irfan',
    maintainer_email='irfanflw@todo.com',
    description='ROS 2 modem link monitor.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'modem_monitor = comms_modem_monitor.modem_monitor:main',
        ],
    },
)
