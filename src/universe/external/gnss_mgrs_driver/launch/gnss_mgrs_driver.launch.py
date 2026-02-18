import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package share directory
    # CHANGE THIS: 'gnss_mgrs_converter' -> 'gnss_mgrs_driver'
    gnss_mgrs_driver_pkg_share = get_package_share_directory('gnss_mgrs_driver')

    # Declare launch arguments for parameters
    mgrs_grid_code_arg = DeclareLaunchArgument(
        'mgrs_grid_code',
        default_value='54SUE',
        description='MGRS grid code for conversion (e.g., 54SUE for AWSIM LAB)'
    )
    nav_sat_fix_frame_id_arg = DeclareLaunchArgument(
        'nav_sat_fix_frame_id',
        default_value='gnss_link',
        description='Frame ID for the published NavSatFix message'
    )
    nav_sat_fix_status_arg = DeclareLaunchArgument(
        'nav_sat_fix_status',
        default_value='0', # sensor_msgs.msg.NavSatStatus.STATUS_FIX (0)
        description='NavSatFix status: 0=NO_FIX, 1=FIX, 2=STATUS_FIX (with RTK float/fixed)'
    )
    nav_sat_fix_service_arg = DeclareLaunchArgument(
        'nav_sat_fix_service',
        default_value='15', # GPS | GLONASS | COMPASS | GALILEO (1+2+4+8)
        description='NavSatFix service flags (bitwise OR of GPS=1, GLONASS=2, COMPASS=4, GALILEO=8)'
    )
    nav_sat_fix_covariance_type_arg = DeclareLaunchArgument(
        'nav_sat_fix_covariance_type',
        default_value='0', # COVARIANCE_TYPE_UNKNOWN (0)
        description='NavSatFix covariance type: 0=UNKNOWN, 1=APPROXIMATED, 2=DIAGONAL_KNOWN, 3=ALL_KNOWN'
    )
    output_doppler_velocity_topic_arg = DeclareLaunchArgument(
        'output_doppler_velocity_topic',
        default_value='/sensing/gnss/simulated_gnss_doppler_velocity',
        description='Topic name for the published Doppler velocity'
    )

    # Node definition
    gnss_converter_node = Node(
        # CHANGE THIS: 'gnss_mgrs_converter' -> 'gnss_mgrs_driver'
        package='gnss_mgrs_driver', # Your package name
        executable='gnss_mgrs_converter_node', # The name of your executable
        name='gnss_mgrs_converter_node',
        output='screen', # Show console output
        parameters=[
            {'mgrs_grid_code': LaunchConfiguration('mgrs_grid_code')},
            {'nav_sat_fix_frame_id': LaunchConfiguration('nav_sat_fix_frame_id')},
            {'nav_sat_fix_status': LaunchConfiguration('nav_sat_fix_status')},
            {'nav_sat_fix_service': LaunchConfiguration('nav_sat_fix_service')},
            {'nav_sat_fix_covariance_type': LaunchConfiguration('nav_sat_fix_covariance_type')},
            {'output_doppler_velocity_topic': LaunchConfiguration('output_doppler_velocity_topic')},
        ],
        # Optionally remap topics if needed
        # remappings=[
        #     ('/sensing/gnss/pose_with_covariance', '/your/custom/input/pose'),
        #     ('/sensing/gnss/nav_sat_fix', '/your/custom/output/nav_sat_fix'),
        #     ('/awsim/ground_truth/localization/kinematic_state', '/your/custom/input/kinematic_state'),
        # ]
    )

    return LaunchDescription([
        mgrs_grid_code_arg,
        nav_sat_fix_frame_id_arg,
        nav_sat_fix_status_arg,
        nav_sat_fix_service_arg,
        nav_sat_fix_covariance_type_arg,
        output_doppler_velocity_topic_arg,
        gnss_converter_node
    ])