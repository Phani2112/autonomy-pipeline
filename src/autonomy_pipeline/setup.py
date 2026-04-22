from setuptools import setup
from glob import glob
import os

package_name = 'autonomy_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='candidate',
    maintainer_email='candidate@example.com',
    description='Perception -> local costmap pipeline (Track A, intern task).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_grid_node = autonomy_pipeline.obstacle_grid_node:main',
            'fake_sensor_publisher = autonomy_pipeline.fake_sensor_publisher:main',
        ],
    },
)
