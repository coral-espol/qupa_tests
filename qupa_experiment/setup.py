from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qupa_experiment'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Torres',
    maintainer_email='davatorr@espol.edu.ec',
    description='Brutschy et al. (2012) task-allocation experiment for QUPA',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'experiment         = qupa_experiment.experiment_node:main',
            'experiment_classic = qupa_experiment.experiment_node_classic:main',
            'experiment_timer   = qupa_experiment.experiment_timer_node:main',
            'led_test           = qupa_experiment.led_test:main',
            'forward_stop_test  = qupa_experiment.forward_stop_test:main',
            'color_behavior     = qupa_experiment.color_behavior_node:main',
        ],
    },
)
