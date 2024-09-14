from setuptools import find_packages, setup

package_name = 'autocar'

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
    maintainer='root',
    maintainer_email='tanwoeiming@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "initial_pose=autocar.initial_pose:main",
            "trajectory=autocar.trajectory:main",
            "laser_data=autocar.laser_data:main",
            "avoid_obstacle=autocar.avoid_obstacle:main",
            "path_planning=autocar.path_planning:main",
            "autonomous_exploring=autocar.autonomous_exploring:main",
        ],
    },
)
