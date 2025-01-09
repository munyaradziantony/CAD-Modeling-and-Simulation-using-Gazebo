from setuptools import find_packages, setup

package_name = 'simple_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arthavg7',
    maintainer_email='swarnakar.ani24@gmail.com',
    description='Simple Proportional Controller for Project 1',
    license='Apache',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proportional_controller = simple_controller.controller:main'
        ],
    },
)
