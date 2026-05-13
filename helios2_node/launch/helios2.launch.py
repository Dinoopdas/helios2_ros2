"""
helios2.launch.py — accuracy-tuned launch for the Helios2+ at 300-1250mm.

Quick reference for tuning live (in another terminal):

    ros2 param set /helios2_node image_accumulation 32     # max averaging
    ros2 param set /helios2_node confidence_threshold 1000 # stricter filter
    ros2 param set /helios2_node roi_min_z 0.30
    ros2 param set /helios2_node roi_max_z 1.25

Note: SDK parameters (operating_mode, conversion_gain, image_accumulation, etc.)
only apply at startup. Driver-side parameters (ROI, min_intensity) apply live.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ===== Launch arguments =====
    args = [
        DeclareLaunchArgument("serial_number",     default_value=""),
        DeclareLaunchArgument("frame_id",          default_value="helios2_optical_frame"),
        DeclareLaunchArgument("pointcloud_topic",  default_value="helios2/points"),
        DeclareLaunchArgument("intensity_topic",   default_value="helios2/intensity"),
        DeclareLaunchArgument("publish_intensity", default_value="true"),

        # Helios SDK settings
        DeclareLaunchArgument("operating_mode",     default_value="Distance1250mmSingleFreq",
                              description="Close-range high-precision mode"),
        DeclareLaunchArgument("exposure_selector",  default_value="Exp250Us",
                              description="Exp62_5Us | Exp250Us | Exp1000Us (longest)"),
        DeclareLaunchArgument("conversion_gain",    default_value="High",
                              description="Low | High (HCG for close-range)"),
        DeclareLaunchArgument("image_accumulation", default_value="1",
                              description="1..32 frames averaged in HW. 16 = good balance. 32 = best quality."),
        DeclareLaunchArgument("spatial_filter",     default_value="true"),
        DeclareLaunchArgument("flying_pixels_remove", default_value="true"),
        DeclareLaunchArgument("confidence_threshold", default_value="500",
                              description="0..65535. Higher = stricter."),
        DeclareLaunchArgument("amplitude_gain",     default_value="1.0",
                              description="0..128. Boost for low-reflectance targets."),

        # Driver-side filters
        DeclareLaunchArgument("min_intensity",      default_value="0"),
        DeclareLaunchArgument("invalid_as_nan",     default_value="true"),

        # ROI in meters
        DeclareLaunchArgument("roi_min_x", default_value="-1.0"),
        DeclareLaunchArgument("roi_max_x", default_value="1.0"),
        DeclareLaunchArgument("roi_min_y", default_value="-1.0"),
        DeclareLaunchArgument("roi_max_y", default_value="1.0"),
        DeclareLaunchArgument("roi_min_z", default_value="0.20"),
        DeclareLaunchArgument("roi_max_z", default_value="1.30"),
    ]

    helios_node = Node(
        package="helios2_node",
        executable="helios2_node",
        name="helios2_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "serial_number":         LaunchConfiguration("serial_number"),
            "frame_id":              LaunchConfiguration("frame_id"),
            "pointcloud_topic":      LaunchConfiguration("pointcloud_topic"),
            "intensity_topic":       LaunchConfiguration("intensity_topic"),
            "publish_intensity":     LaunchConfiguration("publish_intensity"),
            "operating_mode":        LaunchConfiguration("operating_mode"),
            "exposure_selector":     LaunchConfiguration("exposure_selector"),
            "conversion_gain":       LaunchConfiguration("conversion_gain"),
            "image_accumulation":    LaunchConfiguration("image_accumulation"),
            "spatial_filter":        LaunchConfiguration("spatial_filter"),
            "flying_pixels_remove":  LaunchConfiguration("flying_pixels_remove"),
            "confidence_threshold":  LaunchConfiguration("confidence_threshold"),
            "amplitude_gain":        LaunchConfiguration("amplitude_gain"),
            "min_intensity":         LaunchConfiguration("min_intensity"),
            "invalid_as_nan":        LaunchConfiguration("invalid_as_nan"),
            "roi_min_x":             LaunchConfiguration("roi_min_x"),
            "roi_max_x":             LaunchConfiguration("roi_max_x"),
            "roi_min_y":             LaunchConfiguration("roi_min_y"),
            "roi_max_y":             LaunchConfiguration("roi_max_y"),
            "roi_min_z":             LaunchConfiguration("roi_min_z"),
            "roi_max_z":             LaunchConfiguration("roi_max_z"),
            "image_timeout_ms":      5000,
        }],
    )

    return LaunchDescription([*args, helios_node])
