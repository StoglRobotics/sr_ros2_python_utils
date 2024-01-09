from setuptools import setup

package_name = 'sr_ros2_python_utils'

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
    maintainer='Dr. Denis Štogl',
    maintainer_email='denis@stoglrobotics.de',
    description='Python Utils for ROS 2 you might find useful!',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            "calculate_point_transformed = sr_ros2_python_utils.calculate_point_transformed:trasform_base",
            "trasform_frame = sr_ros2_python_utils.calculate_point_transformed:transform_frame",
        ],
    },
)
