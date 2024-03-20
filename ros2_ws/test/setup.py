from setuptools import find_packages, setup

package_name = 'ros2subscriber'

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
    maintainer='admirm',
    maintainer_email='metaadmir@gmail.com',
    description='Package which reads ROV2 bags and converts the data to CSV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_listener = ros2subscriber.imu_subscriber:main',
            'dvl_listener = ros2subscriber.dvl_subscriber:main',
            'pressure_listener = ros2subscriber.pressure_subscriber:main',
            'sonar_listener = ros2subscriber.sonar_subscriber:main',
        ],
    },
)
