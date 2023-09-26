from setuptools import find_packages, setup

package_name = 'ros2grpc'

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
    maintainer='Yong Jin, Cho',
    maintainer_email='drajin.cho@bosornd.com',
    description='ROS2 and gRPC example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = ros2grpc.server:main',
            'client = ros2grpc.client:main',
        ],
    },
)
