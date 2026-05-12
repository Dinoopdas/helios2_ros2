"""
Launch helios2_node with default parameters.

Usage:
    ros2 launch helios2_node helios2.launch.py
    ros2 launch helios2_node helios2.launch.py operating_mode:=Distance6000mmSingleFreq
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------- Launch arguments -------
    serial_number = DeclareLaunchArgument(
        "serial_number", default_value="",
        description="Camera serial number. Empty = pick first device.")

    frame_id = DeclareLaunchArgument(
        "frame_id", default_value="helios2_optical_frame",
        description="TF frame_id for published clouds and images.")

    operating_mode = DeclareLaunchArgument(
        "operating_mode", default_value="Distance3000mmSingleFreq",
        description=("Helios2 operating mode. Common values: "
                     "Distance1250mmSingleFreq, Distance3000mmSingleFreq, "
                     "Distance6000mmSingleFreq, Distance8300mmMultiFreq."))

    exposure = DeclareLaunchArgument(
        "exposure_time_selector", default_value="Exp1000Us",
        description="Exposure preset: Exp62_5Us, Exp250Us, Exp1000Us.")

    publish_intensity = DeclareLaunchArgument(
        "publish_intensity", default_value="true",
        description="If true, also publish mono16 intensity image.")

    pointcloud_topic = DeclareLaunchArgument(
        "pointcloud_topic", default_value="helios2/points")

    intensity_topic = DeclareLaunchArgument(
        "intensity_topic", default_value="helios2/intensity")

    # ------- Node -------
    helios_node = Node(
        package="helios2_node",
        executable="helios2_node",
        name="helios2_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "serial_number":          LaunchConfiguration("serial_number"),
            "frame_id":               LaunchConfiguration("frame_id"),
            "operating_mode":         LaunchConfiguration("operating_mode"),
            "exposure_time_selector": LaunchConfiguration("exposure_time_selector"),
            "publish_intensity":      LaunchConfiguration("publish_intensity"),
            "pointcloud_topic":       LaunchConfiguration("pointcloud_topic"),
            "intensity_topic":        LaunchConfiguration("intensity_topic"),
            "invalid_as_nan":         True,
            "image_timeout_ms":       2000,
        }],
    )

    return LaunchDescription([
        serial_number, frame_id, operating_mode, exposure,
        publish_intensity, pointcloud_topic, intensity_topic,
        helios_node,
    ])
