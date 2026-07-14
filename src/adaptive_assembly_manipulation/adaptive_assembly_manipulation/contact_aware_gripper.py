"""Pure contact-aware gripper-close state and result evaluation."""

from dataclasses import dataclass, field
from enum import Enum
import threading
from typing import Optional, Tuple


class GripperCloseResult(str, Enum):
    """Stable classifications for a physical gripper close attempt."""

    SUCCESS = 'success'
    CONTACT_LIMITED_SUCCESS = 'contact_limited_success'
    GOAL_REJECTED = 'goal_rejected'
    ACTION_ABORTED = 'action_aborted'
    ACTION_CANCELED = 'action_canceled'
    ACTION_TIMEOUT = 'action_timeout'
    CONTACT_TIMEOUT = 'contact_timeout'
    CONTACT_STALE = 'contact_stale'
    UNILATERAL_CONTACT = 'unilateral_contact'
    WRONG_OBJECT_CONTACT = 'wrong_object_contact'
    NO_TARGET_CONTACT = 'no_target_contact'
    INTERNAL_ERROR = 'internal_error'


class ActionTerminalState(str, Enum):
    """Action terminal states relevant to one gripper command."""

    SUCCEEDED = 'succeeded'
    ABORTED = 'aborted'
    CANCELED = 'canceled'
    TIMEOUT = 'timeout'
    UNKNOWN = 'unknown'


class ContactState(str, Enum):
    """Contact evidence classifications for the current close operation."""

    NO_SAMPLES = 'no_samples'
    NO_TARGET = 'no_target'
    STALE = 'stale'
    UNILATERAL = 'unilateral'
    WRONG_OBJECT = 'wrong_object'
    BILATERAL_SETTLING = 'bilateral_settling'
    BILATERAL_SETTLED = 'bilateral_settled'


@dataclass(frozen=True)
class FingerContactSample:
    """Latest independently timestamped contact state for one finger."""

    received: bool = False
    receipt_stamp_ns: Optional[int] = None
    sensor_stamp_ns: Optional[int] = None
    target_contact: bool = False
    wrong_object_contact: bool = False
    entities: Tuple[str, ...] = ()


@dataclass(frozen=True)
class BilateralContactSnapshot:
    """Latest left and right contact samples from the contact collector."""

    left: FingerContactSample = field(default_factory=FingerContactSample)
    right: FingerContactSample = field(default_factory=FingerContactSample)


@dataclass(frozen=True)
class ContactAssessment:
    """Operation-relative contact evidence and diagnostics."""

    state: ContactState
    left_target_contact: bool
    right_target_contact: bool
    left_fresh: bool
    right_fresh: bool
    left_age_sec: Optional[float]
    right_age_sec: Optional[float]
    settle_duration_sec: float
    left_entities: Tuple[str, ...]
    right_entities: Tuple[str, ...]


@dataclass(frozen=True)
class GripperCloseOutcome:
    """Structured result returned by close-result combination logic."""

    result: GripperCloseResult
    goal_accepted: bool
    action_state: ActionTerminalState
    action_error_code: Optional[int]
    contact: ContactAssessment
    expected_target_object: str
    finger_positions: Tuple[float, ...] = ()
    action_error_string: str = ''
    failure_reason: str = ''

    @property
    def succeeded(self) -> bool:
        """Return true for either permitted close-success classification."""
        return self.result in (
            GripperCloseResult.SUCCESS,
            GripperCloseResult.CONTACT_LIMITED_SUCCESS,
        )


def _sample_age_sec(
    sample: FingerContactSample, now_ns: int
) -> Optional[float]:
    if not sample.received or sample.receipt_stamp_ns is None:
        return None
    return (now_ns - sample.receipt_stamp_ns) / 1.0e9


class BilateralContactValidator:
    """Thread-safe operation-relative freshness and settling evaluator."""

    def __init__(
        self, freshness_timeout_sec: float, settle_duration_sec: float
    ) -> None:
        """Create a validator with strict freshness and settling bounds."""
        if freshness_timeout_sec < 0.0:
            raise ValueError('freshness_timeout_sec must be nonnegative')
        if settle_duration_sec < 0.0:
            raise ValueError('settle_duration_sec must be nonnegative')
        self._freshness_timeout_sec = freshness_timeout_sec
        self._settle_duration_sec = settle_duration_sec
        self._operation_start_ns: Optional[int] = None
        self._settle_start_ns: Optional[int] = None
        self._snapshot = BilateralContactSnapshot()
        self._lock = threading.Lock()

    def start_operation(self, operation_start_ns: int) -> None:
        """Reset per-operation settling without discarding subscriber cache."""
        with self._lock:
            self._operation_start_ns = operation_start_ns
            self._settle_start_ns = None

    def update(
        self, snapshot: BilateralContactSnapshot, now_ns: int
    ) -> ContactAssessment:
        """Store a contact snapshot and return its current assessment."""
        with self._lock:
            self._snapshot = snapshot
            return self._assess_locked(now_ns)

    def assess(self, now_ns: int) -> ContactAssessment:
        """Assess the most recent snapshot at ``now_ns``."""
        with self._lock:
            return self._assess_locked(now_ns)

    def _assess_locked(self, now_ns: int) -> ContactAssessment:
        operation_start_ns = self._operation_start_ns
        left = self._snapshot.left
        right = self._snapshot.right
        left_age = _sample_age_sec(left, now_ns)
        right_age = _sample_age_sec(right, now_ns)

        def fresh(sample: FingerContactSample, age: Optional[float]) -> bool:
            return bool(
                operation_start_ns is not None
                and sample.received
                and sample.receipt_stamp_ns is not None
                and sample.receipt_stamp_ns >= operation_start_ns
                and age is not None
                and 0.0 <= age <= self._freshness_timeout_sec
            )

        left_fresh = fresh(left, left_age)
        right_fresh = fresh(right, right_age)
        left_target = left_fresh and left.target_contact
        right_target = right_fresh and right.target_contact
        wrong_object = (
            (left_fresh and left.wrong_object_contact)
            or (right_fresh and right.wrong_object_contact)
        )

        state = ContactState.NO_TARGET
        settle_duration = 0.0
        if not left.received and not right.received:
            state = ContactState.NO_SAMPLES
        elif wrong_object:
            state = ContactState.WRONG_OBJECT
        elif left_target and right_target:
            if self._settle_start_ns is None:
                self._settle_start_ns = now_ns
            settle_duration = max(
                0.0, (now_ns - self._settle_start_ns) / 1.0e9
            )
            state = (
                ContactState.BILATERAL_SETTLED
                if settle_duration >= self._settle_duration_sec
                else ContactState.BILATERAL_SETTLING
            )
        elif (
            (left.received and not left_fresh)
            or (right.received and not right_fresh)
        ):
            state = ContactState.STALE
        elif left_target != right_target:
            state = ContactState.UNILATERAL

        if state not in (
            ContactState.BILATERAL_SETTLING,
            ContactState.BILATERAL_SETTLED,
        ):
            self._settle_start_ns = None

        return ContactAssessment(
            state=state,
            left_target_contact=left_target,
            right_target_contact=right_target,
            left_fresh=left_fresh,
            right_fresh=right_fresh,
            left_age_sec=left_age,
            right_age_sec=right_age,
            settle_duration_sec=settle_duration,
            left_entities=left.entities,
            right_entities=right.entities,
        )


def evaluate_close_result(
    *,
    goal_accepted: bool,
    action_state: ActionTerminalState,
    action_error_code: Optional[int],
    goal_tolerance_error_code: int,
    contact: ContactAssessment,
    allow_contact_limited_close: bool,
    expected_target_object: str,
    finger_positions: Tuple[float, ...] = (),
    action_error_string: str = '',
) -> GripperCloseOutcome:
    """Combine structured action state and contact evidence deterministically."""
    result = GripperCloseResult.INTERNAL_ERROR
    failure_reason = ''

    if not goal_accepted:
        result = GripperCloseResult.GOAL_REJECTED
    elif action_state == ActionTerminalState.TIMEOUT:
        result = GripperCloseResult.ACTION_TIMEOUT
    elif action_state == ActionTerminalState.CANCELED:
        result = GripperCloseResult.ACTION_CANCELED
    elif action_state == ActionTerminalState.SUCCEEDED:
        if contact.state == ContactState.WRONG_OBJECT:
            result = GripperCloseResult.WRONG_OBJECT_CONTACT
        elif contact.state == ContactState.UNILATERAL:
            result = GripperCloseResult.UNILATERAL_CONTACT
        else:
            result = GripperCloseResult.SUCCESS
    elif action_state == ActionTerminalState.ABORTED:
        if (
            not allow_contact_limited_close
            or action_error_code != goal_tolerance_error_code
        ):
            result = GripperCloseResult.ACTION_ABORTED
        elif contact.state == ContactState.BILATERAL_SETTLED:
            result = GripperCloseResult.CONTACT_LIMITED_SUCCESS
        elif contact.state == ContactState.WRONG_OBJECT:
            result = GripperCloseResult.WRONG_OBJECT_CONTACT
        elif contact.state == ContactState.UNILATERAL:
            result = GripperCloseResult.UNILATERAL_CONTACT
        elif contact.state == ContactState.STALE:
            result = GripperCloseResult.CONTACT_STALE
        elif contact.state == ContactState.NO_TARGET:
            result = GripperCloseResult.NO_TARGET_CONTACT
        else:
            result = GripperCloseResult.CONTACT_TIMEOUT

    if result not in (
        GripperCloseResult.SUCCESS,
        GripperCloseResult.CONTACT_LIMITED_SUCCESS,
    ):
        failure_reason = result.value

    return GripperCloseOutcome(
        result=result,
        goal_accepted=goal_accepted,
        action_state=action_state,
        action_error_code=action_error_code,
        contact=contact,
        expected_target_object=expected_target_object,
        finger_positions=finger_positions,
        action_error_string=action_error_string,
        failure_reason=failure_reason,
    )
