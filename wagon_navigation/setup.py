from setuptools import setup

package_name = 'wagon_navigation'

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
    maintainer='danitech',
    maintainer_email='dahol18@student.sdu.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'articulation_controller = wagon_navigation.articulation_controller:main',
            'world_tf_pub = wagon_navigation.world_pub:main'
        ],
    },
)
