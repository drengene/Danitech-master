from setuptools import setup
import os
from glob import glob

package_name = 'wagon_bringup'
#submodules = "src"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*'))
    ],
    zip_safe=True,
    author='Daniel Holm',
    author_email='dahol18@student.sdu.dk',
    maintainer='Daniel Holm',
    maintainer_email='dahol18@student.sdu.dk',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Bringup for articulated vehicle'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'teleop_keyboard = handirob_teleop.script.teleop_keyboard:main',
            #'teleop_joy = handirob_teleop.script.teleop_joy:main'   
        ],
    },
)
