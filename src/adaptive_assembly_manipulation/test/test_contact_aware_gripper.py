"""Unit tests for bilateral contact and close-result evaluation."""

from adaptive_assembly_manipulation.contact_aware_gripper import (
    ActionTerminalState,
    BilateralContactSnapshot,
    BilateralContactValidator,
    ContactState,
    evaluate_close_result,
    FingerContactSample,
    GripperCloseResult,
)
import pytest


SECOND = 1_000_000_000
GOAL_TOLERANCE_VIOLATED = -5


def _sample(
    stamp_ns,
    *,
    target=False,
    wrong=False,
    received=True,
    entity='target_object::link::collision',
):
    return FingerContactSample(
        received=received,
        receipt_stamp_ns=stamp_ns,
        target_contact=target,
        wrong_object_contact=wrong,
        entities=(entity,) if entity else (),
    )


def _assessment(
    *, left, right, now_ns=2 * SECOND, start_ns=SECOND, settle_sec=0.0
):
    validator = BilateralContactValidator(0.25, settle_sec)
    validator.start_operation(start_ns)
    return validator.update(
        BilateralContactSnapshot(left=left, right=right), now_ns
    )


@pytest.mark.parametrize(
    'left_stamp,right_stamp,expected',
    [
        (1_900_000_000, 1_900_000_000, ContactState.BILATERAL_SETTLED),
        (1_700_000_000, 1_900_000_000, ContactState.STALE),
        (1_900_000_000, 1_700_000_000, ContactState.STALE),
        (1_700_000_000, 1_700_000_000, ContactState.STALE),
        (None, None, ContactState.NO_SAMPLES),
        (0, 0, ContactState.STALE),
        (900_000_000, 1_900_000_000, ContactState.STALE),
    ],
)
def test_contact_freshness_and_operation_start(
    left_stamp, right_stamp, expected
):
    """Classify independent stale, absent, zero-time, and pre-start samples."""
    left = (
        FingerContactSample()
        if left_stamp is None else _sample(left_stamp, target=True)
    )
    right = (
        FingerContactSample()
        if right_stamp is None else _sample(right_stamp, target=True)
    )
    assert _assessment(left=left, right=right).state == expected


@pytest.mark.parametrize(
    'left,right,expected',
    [
        (_sample(1_900_000_000, target=True),
         _sample(1_900_000_000, target=True), ContactState.BILATERAL_SETTLED),
        (_sample(1_900_000_000, target=True),
         _sample(1_900_000_000), ContactState.UNILATERAL),
        (_sample(1_900_000_000),
         _sample(1_900_000_000, target=True), ContactState.UNILATERAL),
        (_sample(1_900_000_000, target=True),
         _sample(1_900_000_000, wrong=True), ContactState.WRONG_OBJECT),
        (_sample(1_900_000_000, wrong=True),
         _sample(1_900_000_000, wrong=True), ContactState.WRONG_OBJECT),
        (_sample(1_900_000_000, target=True, wrong=True),
         _sample(1_900_000_000, target=True), ContactState.WRONG_OBJECT),
    ],
)
def test_bilateral_contact_classification(left, right, expected):
    """Require bilateral target contact and conservatively reject extras."""
    assert _assessment(left=left, right=right).state == expected


def test_contact_loss_resets_settling_interval():
    """Require continuous bilateral contact for the full settling duration."""
    validator = BilateralContactValidator(0.25, 0.20)
    validator.start_operation(SECOND)
    bilateral = BilateralContactSnapshot(
        left=_sample(1_100_000_000, target=True),
        right=_sample(1_100_000_000, target=True),
    )
    assert validator.update(bilateral, 1_100_000_000).state == (
        ContactState.BILATERAL_SETTLING
    )
    lost = BilateralContactSnapshot(
        left=_sample(1_150_000_000, target=True),
        right=_sample(1_150_000_000),
    )
    assert validator.update(lost, 1_150_000_000).state == (
        ContactState.UNILATERAL
    )
    reacquired = BilateralContactSnapshot(
        left=_sample(1_200_000_000, target=True),
        right=_sample(1_200_000_000, target=True),
    )
    assert validator.update(reacquired, 1_200_000_000).state == (
        ContactState.BILATERAL_SETTLING
    )
    settled = BilateralContactSnapshot(
        left=_sample(1_410_000_000, target=True),
        right=_sample(1_410_000_000, target=True),
    )
    assert validator.update(settled, 1_410_000_000).state == (
        ContactState.BILATERAL_SETTLED
    )


@pytest.mark.parametrize(
    'accepted,action_state,contact_state,allowed,expected',
    [
        (True, ActionTerminalState.SUCCEEDED, ContactState.NO_TARGET,
         True, GripperCloseResult.SUCCESS),
        (True, ActionTerminalState.SUCCEEDED, ContactState.BILATERAL_SETTLED,
         True, GripperCloseResult.SUCCESS),
        (True, ActionTerminalState.SUCCEEDED, ContactState.UNILATERAL,
         True, GripperCloseResult.UNILATERAL_CONTACT),
        (True, ActionTerminalState.SUCCEEDED, ContactState.WRONG_OBJECT,
         True, GripperCloseResult.WRONG_OBJECT_CONTACT),
        (True, ActionTerminalState.ABORTED, ContactState.BILATERAL_SETTLED,
         True, GripperCloseResult.CONTACT_LIMITED_SUCCESS),
        (True, ActionTerminalState.ABORTED, ContactState.NO_TARGET,
         True, GripperCloseResult.NO_TARGET_CONTACT),
        (False, ActionTerminalState.ABORTED, ContactState.BILATERAL_SETTLED,
         True, GripperCloseResult.GOAL_REJECTED),
        (True, ActionTerminalState.CANCELED, ContactState.BILATERAL_SETTLED,
         True, GripperCloseResult.ACTION_CANCELED),
        (True, ActionTerminalState.TIMEOUT, ContactState.BILATERAL_SETTLED,
         True, GripperCloseResult.ACTION_TIMEOUT),
        (True, ActionTerminalState.ABORTED, ContactState.UNILATERAL,
         True, GripperCloseResult.UNILATERAL_CONTACT),
        (True, ActionTerminalState.ABORTED, ContactState.STALE,
         True, GripperCloseResult.CONTACT_STALE),
        (True, ActionTerminalState.ABORTED, ContactState.WRONG_OBJECT,
         True, GripperCloseResult.WRONG_OBJECT_CONTACT),
        (True, ActionTerminalState.ABORTED, ContactState.BILATERAL_SETTLED,
         False, GripperCloseResult.ACTION_ABORTED),
    ],
)
def test_action_and_contact_result_table(
    accepted, action_state, contact_state, allowed, expected
):
    """Preserve strict action semantics around contact-limited success."""
    assessment = _assessment_for_state(contact_state)
    outcome = evaluate_close_result(
        goal_accepted=accepted,
        action_state=action_state,
        action_error_code=GOAL_TOLERANCE_VIOLATED,
        goal_tolerance_error_code=GOAL_TOLERANCE_VIOLATED,
        contact=assessment,
        allow_contact_limited_close=allowed,
        expected_target_object='target_object',
    )
    assert outcome.result == expected


def _assessment_for_state(state):
    samples = {
        ContactState.BILATERAL_SETTLED: (
            _sample(1_900_000_000, target=True),
            _sample(1_900_000_000, target=True),
        ),
        ContactState.UNILATERAL: (
            _sample(1_900_000_000, target=True),
            _sample(1_900_000_000),
        ),
        ContactState.STALE: (
            _sample(1_700_000_000, target=True),
            _sample(1_700_000_000, target=True),
        ),
        ContactState.WRONG_OBJECT: (
            _sample(1_900_000_000, wrong=True),
            _sample(1_900_000_000, target=True),
        ),
        ContactState.NO_TARGET: (
            _sample(1_900_000_000),
            _sample(1_900_000_000),
        ),
    }
    left, right = samples[state]
    return _assessment(left=left, right=right)


def test_non_tolerance_abort_is_not_contact_limited_success():
    """Do not reinterpret unrelated action abort codes as physical contact."""
    contact = _assessment_for_state(ContactState.BILATERAL_SETTLED)
    outcome = evaluate_close_result(
        goal_accepted=True,
        action_state=ActionTerminalState.ABORTED,
        action_error_code=-4,
        goal_tolerance_error_code=GOAL_TOLERANCE_VIOLATED,
        contact=contact,
        allow_contact_limited_close=True,
        expected_target_object='target_object',
    )
    assert outcome.result == GripperCloseResult.ACTION_ABORTED
