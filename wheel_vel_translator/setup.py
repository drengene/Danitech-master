from setuptools import setup

package_name = 'wheel_vel_translator'

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
    maintainer_email='Daniel Holm',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_vel_translator = wheel_vel_translator.wheel_vel_translator:main'
        ],
    },
)
