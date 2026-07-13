"""Unit tests for the reusable robot-model parity diagnostic."""

import json

from adaptive_assembly_sim.robot_model_parity import (
    compare_models,
    JointSample,
    main,
    ParitySetupError,
    parse_urdf,
)
import pytest


def _urdf(
    *,
    joint_name='joint1',
    joint_type='revolute',
    parent='base',
    child='arm',
    origin_xyz='0 0 0.5',
    origin_rpy='0 0 0',
    axis='0 0 1',
    lower='-1.0',
    upper='1.0',
    include_joint=True,
    include_tool=True,
    extra_chain_joint=True,
    tool_origin_xyz='0 0 0.2',
):
    """Build a compact expanded URDF fixture."""
    links = {'base', 'arm', 'tool', parent, child}
    if not include_tool:
        links.remove('tool')
    link_xml = ''.join(
        f'<link name="{name}"/>' for name in sorted(links)
    )
    joint_xml = ''
    if include_joint:
        limit = ''
        if joint_type != 'fixed':
            limit = (
                f'<axis xyz="{axis}"/>'
                f'<limit lower="{lower}" upper="{upper}" '
                'effort="10" velocity="2"/>'
            )
        joint_xml += (
            f'<joint name="{joint_name}" type="{joint_type}">'
            f'<parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{origin_xyz}" rpy="{origin_rpy}"/>'
            f'{limit}</joint>'
        )
    if extra_chain_joint and include_tool and child != 'tool':
        joint_xml += (
            '<joint name="tool_fixed" type="fixed">'
            f'<parent link="{child}"/><child link="tool"/>'
            f'<origin xyz="{tool_origin_xyz}" rpy="0 0 0"/>'
            '</joint>'
        )
    return f'<robot name="fixture">{link_xml}{joint_xml}</robot>'


def _compare(reference_xml, candidate_xml, samples=()):
    return compare_models(
        parse_urdf(reference_xml, 'reference'),
        parse_urdf(candidate_xml, 'candidate'),
        reference_base_link='base',
        candidate_base_link='base',
        reference_tool_link='tool',
        candidate_tool_link='tool',
        arm_joints=('joint1',),
        samples=samples,
    )


def _categories(result):
    return {mismatch.category for mismatch in result.structural_mismatches}


def test_identical_chains_pass():
    """Accept identical required joints and base-to-tool topology."""
    result = _compare(_urdf(), _urdf())

    assert result.passed is True
    assert not result.structural_mismatches


def test_missing_joint_is_detected():
    """Identify a required joint absent from the candidate model."""
    result = _compare(_urdf(), _urdf(include_joint=False))

    assert result.passed is False
    assert 'missing_joint' in _categories(result)


def test_changed_joint_type_is_detected():
    """Report a required joint that changes motion type."""
    result = _compare(_urdf(), _urdf(joint_type='fixed'))

    assert 'joint_type_mismatch' in _categories(result)


@pytest.mark.parametrize(
    ('candidate', 'expected_category'),
    [
        (
            _urdf(parent='candidate_base'),
            'parent_link_mismatch',
        ),
        (
            _urdf(child='candidate_arm'),
            'child_link_mismatch',
        ),
    ],
)
def test_changed_joint_parent_or_child_is_detected(
    candidate, expected_category
):
    """Identify both sides of a changed required-joint connection."""
    result = _compare(_urdf(), candidate)

    assert expected_category in _categories(result)


def test_changed_joint_origin_xyz_is_detected():
    """Report a translation mismatch independently from rotation."""
    result = _compare(_urdf(), _urdf(origin_xyz='0.01 0 0.5'))

    assert 'origin_translation_mismatch' in _categories(result)


def test_changed_joint_origin_rpy_is_detected():
    """Report a joint-frame rotation mismatch."""
    result = _compare(_urdf(), _urdf(origin_rpy='0.01 0 0'))

    assert 'origin_rotation_mismatch' in _categories(result)


def test_changed_joint_axis_is_detected():
    """Report different motion axes for a required joint."""
    result = _compare(_urdf(), _urdf(axis='0 1 0'))

    assert 'axis_mismatch' in _categories(result)


def test_changed_joint_limit_is_detected():
    """Report any configured numeric limit outside tolerance."""
    result = _compare(_urdf(), _urdf(upper='0.9'))

    assert 'limit_mismatch' in _categories(result)
    assert any(
        mismatch.field == 'limit.upper'
        for mismatch in result.structural_mismatches
    )


def test_missing_tool_link_is_detected():
    """Distinguish a missing configured tool endpoint."""
    result = _compare(_urdf(), _urdf(include_tool=False))

    assert 'missing_tool_link' in _categories(result)


def test_changed_chain_topology_is_detected():
    """Protect the complete configured base-to-tool chain shape."""
    candidate = _urdf(extra_chain_joint=False, child='tool')
    result = _compare(_urdf(), candidate)

    assert 'child_link_mismatch' in _categories(result)
    assert 'chain_topology_mismatch' in _categories(result)


def test_changed_tool_chain_origin_is_detected():
    """Include fixed TCP-extension transforms in structural parity."""
    result = _compare(_urdf(), _urdf(tool_origin_xyz='0 0 0.21'))

    assert 'origin_translation_mismatch' in _categories(result)
    assert any(
        mismatch.subject.startswith('chain[1]:')
        for mismatch in result.structural_mismatches
    )


def test_fk_identical_models_pass_at_nonzero_sample():
    """Compute matching FK without MoveIt, KDL, or a ROS graph."""
    sample = JointSample('bent', {'joint1': 0.4})
    result = _compare(_urdf(), _urdf(), samples=(sample,))

    assert result.passed is True
    assert result.fk_results[0].position_error_m == pytest.approx(0.0)
    assert result.fk_results[0].orientation_error_rad == pytest.approx(0.0)


def test_fk_different_models_fail_at_nonzero_sample():
    """Turn a Cartesian transform difference into a failed FK sample."""
    sample = JointSample('bent', {'joint1': 0.4})
    result = _compare(
        _urdf(),
        _urdf(origin_xyz='0.02 0 0.5'),
        samples=(sample,),
    )

    assert result.passed is False
    assert result.fk_mismatch_count == 1
    assert result.fk_results[0].position_error_m == pytest.approx(0.02)


def test_malformed_xml_produces_a_parsing_setup_error():
    """Classify malformed XML as setup failure, not model mismatch."""
    with pytest.raises(ParitySetupError, match='failed to parse URDF XML'):
        parse_urdf('<robot><link></robot>', 'broken.urdf')


def test_json_result_contains_required_stable_keys():
    """Protect the top-level machine-readable schema."""
    payload = _compare(_urdf(), _urdf()).to_dict()

    assert set(payload) == {
        'schema_version',
        'passed',
        'sources',
        'configured_links',
        'arm_joints',
        'tolerances',
        'structural_mismatches',
        'fk_samples',
        'mismatch_counts',
    }
    assert set(payload['mismatch_counts']) == {'structural', 'fk', 'total'}
    assert payload['schema_version'] == 1


def test_cli_exit_codes_and_json_output(tmp_path, capsys):
    """Return 0 for parity, 1 for mismatch, and stable JSON on request."""
    reference = tmp_path / 'reference.urdf'
    candidate = tmp_path / 'candidate.urdf'
    reference.write_text(_urdf(), encoding='utf-8')
    candidate.write_text(_urdf(), encoding='utf-8')
    common = [
        str(reference),
        str(candidate),
        '--base-link',
        'base',
        '--tool-link',
        'tool',
        '--arm-joints',
        'joint1',
        '--no-fk',
        '--json',
    ]

    assert main(common) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload['passed'] is True

    candidate.write_text(_urdf(axis='0 1 0'), encoding='utf-8')
    assert main(common) == 1
    payload = json.loads(capsys.readouterr().out)
    assert payload['passed'] is False
    assert payload['mismatch_counts']['structural'] >= 1


def test_cli_setup_error_exit_code_is_two(tmp_path, capsys):
    """Reserve exit 2 for unreadable or malformed diagnostic inputs."""
    malformed = tmp_path / 'malformed.urdf'
    valid = tmp_path / 'valid.urdf'
    malformed.write_text('<robot>', encoding='utf-8')
    valid.write_text(_urdf(), encoding='utf-8')

    exit_code = main([str(malformed), str(valid), '--no-fk'])

    assert exit_code == 2
    assert 'failed to parse URDF XML' in capsys.readouterr().err
