"""Static regression tests for the dedicated physical target pose path."""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
WORLD = (
    ROOT / 'src/adaptive_assembly_sim/worlds'
    / 'adaptive_assembly_physical_workcell.sdf'
)
LAUNCH = (
    ROOT / 'src/adaptive_assembly_bringup/launch'
    / 'adaptive_assembly_physical_pick_place_execution.launch.py'
)


def test_target_model_has_single_pose_publisher_configuration():
    """Publish only one bounded-rate model Pose from the target object."""
    root = ET.parse(WORLD).getroot()
    target = root.find(".//model[@name='target_object']")
    assert target is not None
    plugins = target.findall(
        "plugin[@name='gz::sim::systems::PosePublisher']"
    )
    assert len(plugins) == 1
    plugin = plugins[0]
    assert plugin.get('filename') == 'gz-sim-pose-publisher-system'
    assert plugin.findtext('publish_model_pose') == 'true'
    assert plugin.findtext('use_pose_vector_msg') == 'false'
    assert plugin.findtext('update_frequency') == '30'
    for setting in (
        'publish_link_pose',
        'publish_visual_pose',
        'publish_collision_pose',
        'publish_sensor_pose',
        'publish_nested_model_pose',
    ):
        assert plugin.findtext(setting) == 'false'


def test_physical_launch_bridges_pose_to_raw_pose_stamped():
    """Physical launch opts into PoseStamped without changing Pose_V mode."""
    source = LAUNCH.read_text(encoding='utf-8')
    for token in (
        "'target_object_gazebo_pose_topic': '/model/target_object/pose'",
        "'target_object_raw_pose_topic': '/gazebo_target_object_pose_raw'",
        "'@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'",
        "'input_message_type': 'pose_stamped'",
    ):
        assert token in source
    assert "'@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'" not in source
