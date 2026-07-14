"""Tests for exact Gazebo target matching and contact normalization."""

from types import SimpleNamespace

from adaptive_assembly_execution.gazebo_grasp_contact_status_node import (
    analyze_contact_message,
    entity_name_matches_model,
)
import pytest


@pytest.mark.parametrize(
    'entity_name,expected',
    [
        ('target_object', True),
        ('target_object::link::collision', True),
        ('default::target_object::link::collision', True),
        ('target_object_backup::link::collision', False),
        ('fixture_target_object::link::collision', False),
        ('fixture::target_object_link::collision', False),
        ('fixture::target_object::collision', False),
        ('', False),
        ('::::', False),
    ],
)
def test_exact_scoped_target_model_matching(entity_name, expected):
    """Match only exact scoped tokens, never unrelated substrings."""
    assert entity_name_matches_model(entity_name, 'target_object') is expected


def _contact(other_name):
    return SimpleNamespace(
        collision1=SimpleNamespace(
            name='panda::panda_leftfinger::panda_leftfinger_collision'
        ),
        collision2=SimpleNamespace(name=other_name),
    )


def _message(*contacts):
    return SimpleNamespace(contacts=list(contacts))


def test_target_contact_reports_external_entity():
    """Recognize the intended object and exclude the sensor finger itself."""
    observation = analyze_contact_message(
        _message(_contact('default::target_object::link::collision')),
        'target_object',
        'panda_leftfinger',
        True,
    )
    assert observation.target_contact is True
    assert observation.wrong_object_contact is False
    assert observation.entities == (
        'default::target_object::link::collision',
    )


def test_wrong_and_mixed_contacts_are_conservative_failures():
    """Reject wrong-only and target-plus-unrelated contact samples."""
    wrong = analyze_contact_message(
        _message(_contact('default::table::link::collision')),
        'target_object',
        'panda_leftfinger',
        True,
    )
    assert wrong.target_contact is False
    assert wrong.wrong_object_contact is True

    mixed = analyze_contact_message(
        _message(
            _contact('target_object::link::collision'),
            _contact('assembly_fixture::link::collision'),
        ),
        'target_object',
        'panda_leftfinger',
        True,
    )
    assert mixed.target_contact is True
    assert mixed.wrong_object_contact is True


def test_empty_contact_message_clears_target_state():
    """Treat a fresh empty Contacts sample as contact loss, not cached truth."""
    observation = analyze_contact_message(
        _message(), 'target_object', 'panda_leftfinger', True
    )
    assert observation.target_contact is False
    assert observation.wrong_object_contact is False
    assert observation.reason == 'no_contact'
