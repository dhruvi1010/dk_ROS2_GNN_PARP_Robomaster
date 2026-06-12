from setuptools import find_packages, setup

package_name = 'route_cost_puc_pynode'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='dhruvi koshiya',
    maintainer_email='dhruvikoshiya1010@gmail.com',
    description='Computes route cost J(pi) + PUC for the Nav2 global plan.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'route_cost_puc_node = route_cost_puc_pynode.route_cost_puc_pynode:main',
            'route_cost_csv_logger = route_cost_puc_pynode.route_cost_csv_logger:main',
        ],
    },
)
