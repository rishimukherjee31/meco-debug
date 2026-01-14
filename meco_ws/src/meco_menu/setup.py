from setuptools import setup
import os
from glob import glob

package_name = 'meco_menu'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='MeCO Menu System - Hierarchical menu for ROV operations',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'menu_node = meco_menu.menu_node:main',
            'menu_node_debug = meco_menu.menu_node_debug:main',
        ],
    },
)
