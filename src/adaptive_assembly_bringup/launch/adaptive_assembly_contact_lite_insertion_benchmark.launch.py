"""Launch the contact-lite geometric insertion benchmark."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start known-reachable planning plus geometric insertion evaluation."""
    reachable_sequence = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_sequence_planning_reachable.launch.py',
    ])

    target_pose_topic = LaunchConfiguration('target_pose_topic')
    achieved_pose_topic = LaunchConfiguration('achieved_pose_topic')
    position_tolerance_mm = LaunchConfiguration('position_tolerance_mm')
    orientation_tolerance_deg = LaunchConfiguration(
        'orientation_tolerance_deg'
    )
    require_execution_success = LaunchConfiguration(
        'require_execution_success'
    )
    achieved_pose_source = LaunchConfiguration('achieved_pose_source')
    use_planning_scene_audit = LaunchConfiguration('use_planning_scene_audit')

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_pose_topic',
            default_value='/panda_assembly_pose',
            description='Target insertion pose topic.',
        ),
        DeclareLaunchArgument(
            'achieved_pose_topic',
            default_value='/panda_assembly_pose',
            description=(
                'Achieved insertion pose topic. Defaults to the planned '
                'assembly pose when execution feedback is unavailable.'
            ),
        ),
        DeclareLaunchArgument(
            'position_tolerance_mm',
            default_value='5.0',
            description='Maximum allowed insertion position error in mm.',
        ),
        DeclareLaunchArgument(
            'orientation_tolerance_deg',
            default_value='5.0',
            description='Maximum allowed insertion orientation error in deg.',
        ),
        DeclareLaunchArgument(
            'require_execution_success',
            default_value='false',
            description=(
                'Require /assembly_ros2_control_execution_success=true before '
                'reporting insertion success.'
            ),
        ),
        DeclareLaunchArgument(
            'achieved_pose_source',
            default_value='planned_pose',
            description='Status label for the achieved pose source.',
        ),
        DeclareLaunchArgument(
            'use_planning_scene_audit',
            default_value='true',
            description=(
                'Whether the reachable sequence launch includes audit.'
            ),
        ),
        LogInfo(
            msg='Launching contact-lite insertion benchmark. Execution is '
            'optional and disabled by default; the default achieved pose is '
            'the planned Panda assembly pose.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(reachable_sequence),
            launch_arguments={
                'use_planning_scene_audit': use_planning_scene_audit,
            }.items(),
        ),
        Node(
            package='adaptive_assembly_benchmark',
            executable='contact_lite_insertion_evaluator_node',
            name='contact_lite_insertion_evaluator_node',
            output='screen',
            parameters=[{
                'target_pose_topic': target_pose_topic,
                'achieved_pose_topic': achieved_pose_topic,
                'position_tolerance_mm': position_tolerance_mm,
                'orientation_tolerance_deg': orientation_tolerance_deg,
                'require_execution_success': require_execution_success,
                'achieved_pose_source': achieved_pose_source,
            }],
        ),
    ])
