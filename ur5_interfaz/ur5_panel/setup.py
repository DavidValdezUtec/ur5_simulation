import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ur5_panel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.qss') + glob('config/*.json')),
        (os.path.join('share', package_name, 'resource/icons'), glob('resource/icons/*.svg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david.valdez@utec.edu.pe',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'prueba2 = ur5_panel.prueba2:main',
        ],
    },
)
