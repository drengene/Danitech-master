from setuptools import setup

package_name = 'wheel_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rasmus Junge',
    maintainer_email='mail@raju.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odom = wheel_odometry.wheel_odom:main',
            'odom_tester = wheel_odometry.odom_tester:main',
            'pointcloud = wheel_odometry.pointcloud:main'
        ],
    },
)
