"""Deterministic helpers for the task-level grasp sequence schema."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class GraspCandidate:
    """Numeric representation of one deterministic grasp candidate."""

    index: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def validate_candidate_configuration(count: int, selected_index: int) -> None:
    """Raise ValueError when candidate count or selection is invalid."""
    if count < 1:
        raise ValueError('grasp_candidate_count must be greater than or equal to 1')
    if selected_index < 0 or selected_index >= count:
        raise ValueError(
            'selected_grasp_candidate_index must be in the candidate range'
        )


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Return yaw from a quaternion using the standard ZYX convention."""
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(sin_yaw, cos_yaw)


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return an x, y, z, w yaw-only quaternion."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def generate_grasp_candidates(
    x: float,
    y: float,
    z: float,
    orientation: Sequence[float],
    count: int,
    yaw_step_rad: float,
    preserve_orientation: bool = False,
) -> list[GraspCandidate]:
    """Generate deterministic candidates at one position around target yaw."""
    validate_candidate_configuration(count, 0)
    if not math.isfinite(yaw_step_rad):
        raise ValueError('grasp_candidate_yaw_step_rad must be finite')
    if len(orientation) != 4:
        raise ValueError('orientation must contain qx, qy, qz, qw')

    target_orientation = tuple(float(value) for value in orientation)
    target_yaw = yaw_from_quaternion(*target_orientation)
    candidates = []
    for index in range(count):
        quaternion = (
            target_orientation if preserve_orientation
            else yaw_quaternion(target_yaw + index * yaw_step_rad)
        )
        candidates.append(
            GraspCandidate(index, x, y, z, *quaternion)
        )
    return candidates


def select_candidate(
    candidates: Sequence[GraspCandidate], selected_index: int
) -> GraspCandidate:
    """Return the configured candidate after validating the selection."""
    validate_candidate_configuration(len(candidates), selected_index)
    return candidates[selected_index]


def lift_z(selected_grasp_z: float, height_offset: float) -> float:
    """Return lift height and reject negative offsets."""
    if height_offset < 0.0:
        raise ValueError('lift_height_offset must be greater than or equal to zero')
    return selected_grasp_z + height_offset


def format_grasp_candidates(
    candidates: Iterable[GraspCandidate], selected_index: int, frame_id: str
) -> str:
    """Format candidates as a stable semicolon-delimited schema."""
    values = list(candidates)
    validate_candidate_configuration(len(values), selected_index)
    compact = '|'.join(
        f'{candidate.index}:'
        f'{candidate.x:.6f},{candidate.y:.6f},{candidate.z:.6f},'
        f'{candidate.qx:.6f},{candidate.qy:.6f},{candidate.qz:.6f},'
        f'{candidate.qw:.6f}'
        for candidate in values
    )
    return (
        f'event=grasp_candidates;count={len(values)};'
        f'selected_index={selected_index};frame_id={frame_id};'
        f'candidates={compact}'
    )


def format_grasp_sequence_status(
    status: str,
    selected_index: int,
    candidate_count: int,
    frame_id: str,
    target: Sequence[float],
    selected: Sequence[float],
    lift_height: float,
    object_place: Sequence[float],
    assembly_pose_mode: str,
) -> str:
    """Format a stable simulator-only grasp sequence status event."""
    return (
        f'event=grasp_sequence;status={status};selected_index={selected_index};'
        f'candidate_count={candidate_count};frame_id={frame_id};'
        f'target_x={target[0]:.6f};target_y={target[1]:.6f};'
        f'target_z={target[2]:.6f};selected_x={selected[0]:.6f};'
        f'selected_y={selected[1]:.6f};selected_z={selected[2]:.6f};'
        f'lift_z={lift_height:.6f};object_place_x={object_place[0]:.6f};'
        f'object_place_y={object_place[1]:.6f};'
        f'object_place_z={object_place[2]:.6f};'
        f'assembly_pose_mode={assembly_pose_mode};execution=false;'
        'simulated_only=true;real_hardware=false'
    )
