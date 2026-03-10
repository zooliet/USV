from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uwtec_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('config/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junhyun Shin',
    maintainer_email='hl1sqi@gmail.com',
    description='TODO: Package description',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'demo_node = uwtec_localization.demo_node:main',
            'localizer = uwtec_localization.localizer_afr:main',
            'um982_config = uwtec_localization.um982_configurator:main',

        ],
    },
)
