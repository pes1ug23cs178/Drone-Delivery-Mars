from setuptools import setup

package_name = 'mars_graph_planner'

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
    maintainer='MARS Team',
    maintainer_email='mars@example.com',
    description='Graph-based 3D path planner with performance tracking for MARS drone delivery.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_path_planner = mars_graph_planner.graph_path_planner_node:main',
        ],
    },
)
