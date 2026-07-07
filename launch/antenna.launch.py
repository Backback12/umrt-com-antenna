from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    switch_node = Node(
        package='umrt_com_antenna',
        executable='switch_publisher_node',
        name='switch_publisher_node',
        output='screen',
    )

    imu_node = Node(
        package='umrt_com_antenna',
        executable='mpu9265_node',
        name='mpu9265_node',
        output='screen',
        parameters=[{'i2c_bus': '/dev/i2c-1'}],
    )

    return LaunchDescription([switch_node, imu_node])