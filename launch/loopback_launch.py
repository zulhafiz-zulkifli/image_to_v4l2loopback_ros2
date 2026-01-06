# image_to_v4l2loopback_ros2/launch/loopback_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='/image_raw',
        description='ROS topic to subscribe for image data')

    device_arg = DeclareLaunchArgument(
        'device', default_value='/dev/video55',
        description='Path to video capture device')

    width_arg = DeclareLaunchArgument(
        'width', default_value='640',
        description='Target image width')

    height_arg = DeclareLaunchArgument(
        'height', default_value='480',
        description='Target image height')

    format_arg = DeclareLaunchArgument(
        'format', default_value='YUYV',
        description='Target image pixel format')

    # Define the node action
    stream_node = Node(
        package='image_to_v4l2loopback',  # Your package name
        executable='stream',  # Name of the executable created by stream.cpp
        name='image_stream_node',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'device': LaunchConfiguration('device'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'format': LaunchConfiguration('format')
        }],
    )

    # Create and return the launch description
    return LaunchDescription([
        topic_arg,
        device_arg,
        width_arg,
        height_arg,
        format_arg,
        stream_node
    ])
