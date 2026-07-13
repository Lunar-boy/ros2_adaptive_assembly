"""Compare the kinematic contracts of two expanded URDF robot models."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping, Optional, Sequence
import xml.etree.ElementTree as ElementTree


DEFAULT_ARM_JOINTS = tuple(f'panda_joint{index}' for index in range(1, 8))
DEFAULT_BASE_LINK = 'panda_link0'
DEFAULT_TOOL_LINK = 'panda_link8'
CURRENT_PANDA_TOOL_LINK = DEFAULT_TOOL_LINK
SCHEMA_VERSION = 1


class ParitySetupError(RuntimeError):
    """Raised when model loading or diagnostic configuration is invalid."""


@dataclass(frozen=True)
class Tolerances:
    """Numerical tolerances used by structural and FK comparisons."""

    translation: float = 1e-6
    rotation: float = 1e-6
    axis: float = 1e-6
    joint_limit: float = 1e-6

    def validate(self) -> None:
        """Reject negative or non-finite tolerance values."""
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value < 0.0:
                raise ParitySetupError(
                    f'{name} tolerance must be finite and non-negative'
                )


@dataclass(frozen=True)
class JointData:
    """Kinematic fields extracted from one URDF joint."""

    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: Optional[tuple[float, float, float]]
    limits: Mapping[str, float]


@dataclass(frozen=True)
class UrdfModel:
    """Parsed subset of a URDF needed for parity diagnostics."""

    name: str
    source: str
    links: frozenset[str]
    joints: Mapping[str, JointData]
    child_to_joint: Mapping[str, JointData]


@dataclass(frozen=True)
class MismatchRecord:
    """One actionable structural parity mismatch."""

    category: str
    subject: str
    field: str
    message: str
    reference: Any = None
    candidate: Any = None
    absolute_error: Any = None

    def to_dict(self) -> dict[str, Any]:
        """Return a stable JSON-compatible representation."""
        return asdict(self)


@dataclass(frozen=True)
class JointSample:
    """Named joint configuration used for an FK comparison."""

    name: str
    positions: Mapping[str, float]


@dataclass(frozen=True)
class FkSampleResult:
    """Reference and candidate FK values for one joint sample."""

    name: str
    joint_positions: Mapping[str, float]
    reference_position: tuple[float, float, float]
    candidate_position: tuple[float, float, float]
    position_error_m: float
    orientation_error_rad: float
    passed: bool

    def to_dict(self) -> dict[str, Any]:
        """Return a stable JSON-compatible representation."""
        return asdict(self)


@dataclass(frozen=True)
class ParityResult:
    """Complete structural and forward-kinematics parity result."""

    reference_source: str
    candidate_source: str
    reference_base_link: str
    candidate_base_link: str
    reference_tool_link: str
    candidate_tool_link: str
    arm_joints: tuple[str, ...]
    tolerances: Tolerances
    structural_mismatches: tuple[MismatchRecord, ...] = field(
        default_factory=tuple
    )
    fk_results: tuple[FkSampleResult, ...] = field(default_factory=tuple)

    @property
    def fk_mismatch_count(self) -> int:
        """Count FK samples outside the configured tolerances."""
        return sum(not sample.passed for sample in self.fk_results)

    @property
    def passed(self) -> bool:
        """Return whether both the structural and FK contracts pass."""
        return not self.structural_mismatches and self.fk_mismatch_count == 0

    def to_dict(self) -> dict[str, Any]:
        """Return the stable machine-readable report schema."""
        structural_count = len(self.structural_mismatches)
        fk_count = self.fk_mismatch_count
        return {
            'schema_version': SCHEMA_VERSION,
            'passed': self.passed,
            'sources': {
                'reference': self.reference_source,
                'candidate': self.candidate_source,
            },
            'configured_links': {
                'reference': {
                    'base': self.reference_base_link,
                    'tool': self.reference_tool_link,
                },
                'candidate': {
                    'base': self.candidate_base_link,
                    'tool': self.candidate_tool_link,
                },
            },
            'arm_joints': list(self.arm_joints),
            'tolerances': asdict(self.tolerances),
            'structural_mismatches': [
                mismatch.to_dict()
                for mismatch in self.structural_mismatches
            ],
            'fk_samples': [sample.to_dict() for sample in self.fk_results],
            'mismatch_counts': {
                'structural': structural_count,
                'fk': fk_count,
                'total': structural_count + fk_count,
            },
        }


def _parse_vector(
    text: Optional[str],
    default: tuple[float, float, float],
    context: str,
) -> tuple[float, float, float]:
    if text is None:
        return default
    try:
        values = tuple(float(value) for value in text.split())
    except ValueError as error:
        raise ParitySetupError(f'{context} contains a non-numeric value') from error
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ParitySetupError(f'{context} must contain three finite numbers')
    return values


def parse_urdf(xml_text: str, source: str = '<memory>') -> UrdfModel:
    """Parse an expanded URDF XML document into diagnostic data."""
    try:
        root = ElementTree.fromstring(xml_text)
    except ElementTree.ParseError as error:
        raise ParitySetupError(f'failed to parse URDF XML from {source}: {error}')
    if root.tag != 'robot':
        raise ParitySetupError(f'URDF root in {source} must be <robot>')

    links: set[str] = set()
    for element in root.findall('link'):
        name = element.get('name')
        if not name:
            raise ParitySetupError(f'unnamed link in {source}')
        if name in links:
            raise ParitySetupError(f'duplicate link {name!r} in {source}')
        links.add(name)

    joints: dict[str, JointData] = {}
    child_to_joint: dict[str, JointData] = {}
    for element in root.findall('joint'):
        name = element.get('name')
        joint_type = element.get('type')
        if not name or not joint_type:
            raise ParitySetupError(f'joint without name or type in {source}')
        if name in joints:
            raise ParitySetupError(f'duplicate joint {name!r} in {source}')
        parent_element = element.find('parent')
        child_element = element.find('child')
        parent = parent_element.get('link') if parent_element is not None else None
        child = child_element.get('link') if child_element is not None else None
        if not parent or not child:
            raise ParitySetupError(
                f'joint {name!r} in {source} is missing a parent or child link'
            )
        origin = element.find('origin')
        origin_xyz = _parse_vector(
            origin.get('xyz') if origin is not None else None,
            (0.0, 0.0, 0.0),
            f'joint {name!r} origin xyz',
        )
        origin_rpy = _parse_vector(
            origin.get('rpy') if origin is not None else None,
            (0.0, 0.0, 0.0),
            f'joint {name!r} origin rpy',
        )
        axis_element = element.find('axis')
        axis = None
        if joint_type in {'revolute', 'continuous', 'prismatic'}:
            axis = _parse_vector(
                axis_element.get('xyz') if axis_element is not None else None,
                (1.0, 0.0, 0.0),
                f'joint {name!r} axis',
            )
        limit_element = element.find('limit')
        limits: dict[str, float] = {}
        if limit_element is not None:
            for key in sorted(limit_element.attrib):
                try:
                    value = float(limit_element.attrib[key])
                except ValueError as error:
                    raise ParitySetupError(
                        f'joint {name!r} limit {key!r} is not numeric'
                    ) from error
                if not math.isfinite(value):
                    raise ParitySetupError(
                        f'joint {name!r} limit {key!r} is not finite'
                    )
                limits[key] = value
        joint = JointData(
            name=name,
            joint_type=joint_type,
            parent=parent,
            child=child,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            axis=axis,
            limits=limits,
        )
        if child in child_to_joint:
            raise ParitySetupError(
                f'link {child!r} has multiple parent joints in {source}'
            )
        joints[name] = joint
        child_to_joint[child] = joint

    return UrdfModel(
        name=root.get('name', ''),
        source=source,
        links=frozenset(links),
        joints=joints,
        child_to_joint=child_to_joint,
    )


def expand_urdf_source(
    source: Path | str,
    xacro_args: Sequence[str] = (),
) -> str:
    """Read URDF XML or expand a xacro source with the installed executable."""
    path = Path(source).expanduser()
    if not path.is_file():
        raise ParitySetupError(f'robot model source does not exist: {path}')
    if path.name.endswith('.xacro'):
        command = ['xacro', str(path), *xacro_args]
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
            )
        except FileNotFoundError as error:
            raise ParitySetupError(
                'xacro executable is unavailable; install ros-jazzy-xacro'
            ) from error
        if completed.returncode != 0:
            detail = completed.stderr.strip() or completed.stdout.strip()
            raise ParitySetupError(
                f'xacro expansion failed for {path}: {detail}'
            )
        return completed.stdout
    try:
        return path.read_text(encoding='utf-8')
    except OSError as error:
        raise ParitySetupError(f'failed to read {path}: {error}') from error


def load_urdf_model(
    source: Path | str,
    xacro_args: Sequence[str] = (),
) -> UrdfModel:
    """Load and parse an expanded URDF or xacro file."""
    path = Path(source).expanduser()
    return parse_urdf(expand_urdf_source(path, xacro_args), str(path))


def _absolute_errors(
    reference: Sequence[float], candidate: Sequence[float]
) -> tuple[float, ...]:
    return tuple(abs(left - right) for left, right in zip(reference, candidate))


def _chain(model: UrdfModel, base_link: str, tool_link: str) -> list[JointData]:
    current = tool_link
    reversed_chain: list[JointData] = []
    visited: set[str] = set()
    while current != base_link:
        if current in visited:
            raise ParitySetupError(
                f'cycle detected while tracing {model.source} from '
                f'{tool_link} to {base_link}'
            )
        visited.add(current)
        joint = model.child_to_joint.get(current)
        if joint is None:
            raise ParitySetupError(
                f'no chain exists in {model.source} from base {base_link!r} '
                f'to tool {tool_link!r}'
            )
        reversed_chain.append(joint)
        current = joint.parent
    return list(reversed(reversed_chain))


def _chain_signature(
    chain: Sequence[JointData], arm_joints: Sequence[str]
) -> list[str]:
    required = set(arm_joints)
    fixed_index = 0
    signature: list[str] = []
    for joint in chain:
        if joint.name in required:
            signature.append(joint.name)
        else:
            signature.append(f'<extra:{fixed_index}:{joint.joint_type}>')
            fixed_index += 1
    return signature


def _compare_chain_extension_kinematics(
    reference_chain: Sequence[JointData],
    candidate_chain: Sequence[JointData],
    arm_joints: Sequence[str],
    tolerances: Tolerances,
) -> list[MismatchRecord]:
    """Compare non-arm joints that map configured arm links to each tool."""
    required = set(arm_joints)
    mismatches: list[MismatchRecord] = []
    for index, (reference_joint, candidate_joint) in enumerate(zip(
        reference_chain, candidate_chain
    )):
        if reference_joint.name in required and candidate_joint.name in required:
            continue
        subject = (
            f'chain[{index}]:'
            f'{reference_joint.name}|{candidate_joint.name}'
        )
        vector_fields = (
            (
                'origin_xyz', 'origin_translation_mismatch',
                tolerances.translation,
            ),
            (
                'origin_rpy', 'origin_rotation_mismatch',
                tolerances.rotation,
            ),
            ('axis', 'axis_mismatch', tolerances.axis),
        )
        for field_name, category, tolerance in vector_fields:
            reference_value = getattr(reference_joint, field_name)
            candidate_value = getattr(candidate_joint, field_name)
            if reference_value is None or candidate_value is None:
                if reference_value != candidate_value:
                    mismatches.append(MismatchRecord(
                        category=category,
                        subject=subject,
                        field=field_name,
                        message=f'{subject} {field_name} mismatch',
                        reference=reference_value,
                        candidate=candidate_value,
                    ))
                continue
            errors = _absolute_errors(reference_value, candidate_value)
            if any(error > tolerance for error in errors):
                mismatches.append(MismatchRecord(
                    category=category,
                    subject=subject,
                    field=field_name,
                    message=f'{subject} {field_name} mismatch',
                    reference=list(reference_value),
                    candidate=list(candidate_value),
                    absolute_error=list(errors),
                ))
        for limit_name in sorted(
            set(reference_joint.limits) | set(candidate_joint.limits)
        ):
            reference_limit = reference_joint.limits.get(limit_name)
            candidate_limit = candidate_joint.limits.get(limit_name)
            if reference_limit is None or candidate_limit is None:
                mismatches.append(MismatchRecord(
                    category='limit_mismatch',
                    subject=subject,
                    field=f'limit.{limit_name}',
                    message=f'{subject} limit {limit_name} mismatch',
                    reference=reference_limit,
                    candidate=candidate_limit,
                ))
                continue
            error = abs(reference_limit - candidate_limit)
            if error > tolerances.joint_limit:
                mismatches.append(MismatchRecord(
                    category='limit_mismatch',
                    subject=subject,
                    field=f'limit.{limit_name}',
                    message=f'{subject} limit {limit_name} mismatch',
                    reference=reference_limit,
                    candidate=candidate_limit,
                    absolute_error=error,
                ))
    return mismatches


def compare_structure(
    reference: UrdfModel,
    candidate: UrdfModel,
    *,
    reference_base_link: str,
    candidate_base_link: str,
    reference_tool_link: str,
    candidate_tool_link: str,
    arm_joints: Sequence[str],
    tolerances: Tolerances,
) -> tuple[MismatchRecord, ...]:
    """Compare configured links, required joints, and chain topology."""
    mismatches: list[MismatchRecord] = []
    link_checks = (
        ('reference', reference, reference_base_link, 'base'),
        ('reference', reference, reference_tool_link, 'tool'),
        ('candidate', candidate, candidate_base_link, 'base'),
        ('candidate', candidate, candidate_tool_link, 'tool'),
    )
    missing_endpoint = False
    for side, model, link, role in link_checks:
        if link not in model.links:
            missing_endpoint = True
            mismatches.append(MismatchRecord(
                category=f'missing_{role}_link',
                subject=link,
                field=role,
                message=f'{side} model is missing configured {role} link {link!r}',
                reference=link if side == 'reference' else None,
                candidate=link if side == 'candidate' else None,
            ))

    for joint_name in arm_joints:
        reference_joint = reference.joints.get(joint_name)
        candidate_joint = candidate.joints.get(joint_name)
        if reference_joint is None or candidate_joint is None:
            missing_from = []
            if reference_joint is None:
                missing_from.append('reference')
            if candidate_joint is None:
                missing_from.append('candidate')
            mismatches.append(MismatchRecord(
                category='missing_joint',
                subject=joint_name,
                field='joint',
                message=(
                    f'joint {joint_name!r} is missing from '
                    + ' and '.join(missing_from)
                    + ' model'
                ),
                reference=reference_joint is not None,
                candidate=candidate_joint is not None,
            ))
            continue

        scalar_fields = (
            ('joint_type', 'joint_type_mismatch'),
            ('parent', 'parent_link_mismatch'),
            ('child', 'child_link_mismatch'),
        )
        for field_name, category in scalar_fields:
            reference_value = getattr(reference_joint, field_name)
            candidate_value = getattr(candidate_joint, field_name)
            if reference_value != candidate_value:
                mismatches.append(MismatchRecord(
                    category=category,
                    subject=joint_name,
                    field=field_name,
                    message=f'{joint_name} {field_name} mismatch',
                    reference=reference_value,
                    candidate=candidate_value,
                ))

        vector_fields = (
            (
                'origin_xyz', 'origin_translation_mismatch',
                tolerances.translation,
            ),
            (
                'origin_rpy', 'origin_rotation_mismatch',
                tolerances.rotation,
            ),
            ('axis', 'axis_mismatch', tolerances.axis),
        )
        for field_name, category, tolerance in vector_fields:
            reference_value = getattr(reference_joint, field_name)
            candidate_value = getattr(candidate_joint, field_name)
            if reference_value is None or candidate_value is None:
                if reference_value != candidate_value:
                    mismatches.append(MismatchRecord(
                        category=category,
                        subject=joint_name,
                        field=field_name,
                        message=f'{joint_name} {field_name} mismatch',
                        reference=reference_value,
                        candidate=candidate_value,
                    ))
                continue
            errors = _absolute_errors(reference_value, candidate_value)
            if any(error > tolerance for error in errors):
                mismatches.append(MismatchRecord(
                    category=category,
                    subject=joint_name,
                    field=field_name,
                    message=f'{joint_name} {field_name} mismatch',
                    reference=list(reference_value),
                    candidate=list(candidate_value),
                    absolute_error=list(errors),
                ))

        limit_keys = sorted(
            set(reference_joint.limits) | set(candidate_joint.limits)
        )
        for limit_name in limit_keys:
            reference_limit = reference_joint.limits.get(limit_name)
            candidate_limit = candidate_joint.limits.get(limit_name)
            if reference_limit is None or candidate_limit is None:
                mismatches.append(MismatchRecord(
                    category='limit_mismatch',
                    subject=joint_name,
                    field=f'limit.{limit_name}',
                    message=f'{joint_name} limit {limit_name} mismatch',
                    reference=reference_limit,
                    candidate=candidate_limit,
                ))
            else:
                error = abs(reference_limit - candidate_limit)
                if error > tolerances.joint_limit:
                    mismatches.append(MismatchRecord(
                        category='limit_mismatch',
                        subject=joint_name,
                        field=f'limit.{limit_name}',
                        message=f'{joint_name} limit {limit_name} mismatch',
                        reference=reference_limit,
                        candidate=candidate_limit,
                        absolute_error=error,
                    ))

    if not missing_endpoint:
        try:
            reference_chain = _chain(
                reference, reference_base_link, reference_tool_link
            )
            candidate_chain = _chain(
                candidate, candidate_base_link, candidate_tool_link
            )
        except ParitySetupError as error:
            mismatches.append(MismatchRecord(
                category='chain_topology_mismatch',
                subject='base_to_tool',
                field='chain',
                message=str(error),
            ))
        else:
            reference_signature = _chain_signature(reference_chain, arm_joints)
            candidate_signature = _chain_signature(candidate_chain, arm_joints)
            if reference_signature != candidate_signature:
                mismatches.append(MismatchRecord(
                    category='chain_topology_mismatch',
                    subject='base_to_tool',
                    field='chain',
                    message='base-to-tool chain topology mismatch',
                    reference=reference_signature,
                    candidate=candidate_signature,
                ))
            else:
                mismatches.extend(_compare_chain_extension_kinematics(
                    reference_chain,
                    candidate_chain,
                    arm_joints,
                    tolerances,
                ))

    return tuple(mismatches)


Matrix = tuple[tuple[float, float, float, float], ...]


def _identity() -> Matrix:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _multiply(left: Matrix, right: Matrix) -> Matrix:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(4))
            for column in range(4)
        )
        for row in range(4)
    )


def _transform(rotation: Sequence[Sequence[float]], xyz: Sequence[float]) -> Matrix:
    return tuple(
        tuple(rotation[row][column] for column in range(3)) + (xyz[row],)
        for row in range(3)
    ) + ((0.0, 0.0, 0.0, 1.0),)


def _rpy_rotation(rpy: Sequence[float]) -> tuple[tuple[float, ...], ...]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _axis_rotation(
    axis: Sequence[float], angle: float
) -> tuple[tuple[float, ...], ...]:
    length = math.sqrt(sum(value * value for value in axis))
    if length <= 1e-15:
        raise ParitySetupError('revolute or continuous joint has a zero axis')
    x, y, z = (value / length for value in axis)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus = 1.0 - cosine
    return (
        (
            cosine + x * x * one_minus,
            x * y * one_minus - z * sine,
            x * z * one_minus + y * sine,
        ),
        (
            y * x * one_minus + z * sine,
            cosine + y * y * one_minus,
            y * z * one_minus - x * sine,
        ),
        (
            z * x * one_minus - y * sine,
            z * y * one_minus + x * sine,
            cosine + z * z * one_minus,
        ),
    )


def _joint_transform(joint: JointData, position: float) -> Matrix:
    origin = _transform(_rpy_rotation(joint.origin_rpy), joint.origin_xyz)
    if joint.joint_type in {'fixed', 'floating', 'planar'}:
        if joint.joint_type != 'fixed':
            raise ParitySetupError(
                f'FK does not support {joint.joint_type!r} joint {joint.name!r}'
            )
        return origin
    if joint.joint_type in {'revolute', 'continuous'}:
        if joint.axis is None:
            raise ParitySetupError(f'joint {joint.name!r} has no axis')
        motion = _transform(_axis_rotation(joint.axis, position), (0.0, 0.0, 0.0))
        return _multiply(origin, motion)
    if joint.joint_type == 'prismatic':
        if joint.axis is None:
            raise ParitySetupError(f'joint {joint.name!r} has no axis')
        length = math.sqrt(sum(value * value for value in joint.axis))
        if length <= 1e-15:
            raise ParitySetupError(f'prismatic joint {joint.name!r} has a zero axis')
        translation = tuple(position * value / length for value in joint.axis)
        return _multiply(origin, _transform(_rpy_rotation((0.0, 0.0, 0.0)), translation))
    raise ParitySetupError(
        f'FK does not support joint type {joint.joint_type!r} on {joint.name!r}'
    )


def forward_kinematics(
    model: UrdfModel,
    base_link: str,
    tool_link: str,
    joint_positions: Mapping[str, float],
) -> Matrix:
    """Compute a base-to-tool homogeneous transform for a URDF tree chain."""
    transform = _identity()
    for joint in _chain(model, base_link, tool_link):
        position = float(joint_positions.get(joint.name, 0.0))
        if not math.isfinite(position):
            raise ParitySetupError(
                f'joint sample value for {joint.name!r} is not finite'
            )
        transform = _multiply(transform, _joint_transform(joint, position))
    return transform


def _validate_samples(
    reference: UrdfModel,
    candidate: UrdfModel,
    arm_joints: Sequence[str],
    samples: Sequence[JointSample],
) -> None:
    for sample in samples:
        missing = [joint for joint in arm_joints if joint not in sample.positions]
        if missing:
            raise ParitySetupError(
                f'FK sample {sample.name!r} is missing joints: '
                + ', '.join(missing)
            )
        unknown = sorted(
            joint for joint in sample.positions
            if joint not in reference.joints or joint not in candidate.joints
        )
        if unknown:
            raise ParitySetupError(
                f'FK sample {sample.name!r} contains joints unavailable in '
                f'both models: {", ".join(unknown)}'
            )
        for joint_name, position in sample.positions.items():
            value = float(position)
            if not math.isfinite(value):
                raise ParitySetupError(
                    f'FK sample {sample.name!r} has non-finite value for '
                    f'{joint_name}'
                )
            reference_limits = reference.joints[joint_name].limits
            candidate_limits = candidate.joints[joint_name].limits
            lower_values = [
                limits['lower'] for limits in (reference_limits, candidate_limits)
                if 'lower' in limits
            ]
            upper_values = [
                limits['upper'] for limits in (reference_limits, candidate_limits)
                if 'upper' in limits
            ]
            lower = max(lower_values) if lower_values else None
            upper = min(upper_values) if upper_values else None
            if lower is not None and upper is not None and lower > upper:
                raise ParitySetupError(
                    f'joint {joint_name!r} has no shared limit interval'
                )
            if lower is not None and value < lower:
                raise ParitySetupError(
                    f'FK sample {sample.name!r} value {value} for {joint_name} '
                    f'is below shared lower limit {lower}'
                )
            if upper is not None and value > upper:
                raise ParitySetupError(
                    f'FK sample {sample.name!r} value {value} for {joint_name} '
                    f'is above shared upper limit {upper}'
                )


def _position(transform: Matrix) -> tuple[float, float, float]:
    return tuple(transform[index][3] for index in range(3))


def _orientation_error(reference: Matrix, candidate: Matrix) -> float:
    relative_trace = sum(
        sum(reference[index][row] * candidate[index][row] for index in range(3))
        for row in range(3)
    )
    cosine = max(-1.0, min(1.0, (relative_trace - 1.0) / 2.0))
    return math.acos(cosine)


def compare_fk(
    reference: UrdfModel,
    candidate: UrdfModel,
    *,
    reference_base_link: str,
    candidate_base_link: str,
    reference_tool_link: str,
    candidate_tool_link: str,
    arm_joints: Sequence[str],
    samples: Sequence[JointSample],
    tolerances: Tolerances,
) -> tuple[FkSampleResult, ...]:
    """Compare base-to-tool FK for deterministic joint samples."""
    _validate_samples(reference, candidate, arm_joints, samples)
    results: list[FkSampleResult] = []
    for sample in samples:
        reference_transform = forward_kinematics(
            reference,
            reference_base_link,
            reference_tool_link,
            sample.positions,
        )
        candidate_transform = forward_kinematics(
            candidate,
            candidate_base_link,
            candidate_tool_link,
            sample.positions,
        )
        reference_position = _position(reference_transform)
        candidate_position = _position(candidate_transform)
        position_error = math.sqrt(sum(
            (left - right) ** 2
            for left, right in zip(reference_position, candidate_position)
        ))
        orientation_error = _orientation_error(
            reference_transform, candidate_transform
        )
        results.append(FkSampleResult(
            name=sample.name,
            joint_positions=dict(sorted(sample.positions.items())),
            reference_position=reference_position,
            candidate_position=candidate_position,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            passed=(
                position_error <= tolerances.translation
                and orientation_error <= tolerances.rotation
            ),
        ))
    return tuple(results)


def compare_models(
    reference: UrdfModel,
    candidate: UrdfModel,
    *,
    reference_base_link: str = DEFAULT_BASE_LINK,
    candidate_base_link: str = DEFAULT_BASE_LINK,
    reference_tool_link: str = DEFAULT_TOOL_LINK,
    candidate_tool_link: str = DEFAULT_TOOL_LINK,
    arm_joints: Sequence[str] = DEFAULT_ARM_JOINTS,
    tolerances: Tolerances = Tolerances(),
    samples: Sequence[JointSample] = (),
) -> ParityResult:
    """Run structural and optional FK comparisons for parsed models."""
    tolerances.validate()
    if not arm_joints or len(set(arm_joints)) != len(arm_joints):
        raise ParitySetupError('arm joint names must be non-empty and unique')
    structural_mismatches = compare_structure(
        reference,
        candidate,
        reference_base_link=reference_base_link,
        candidate_base_link=candidate_base_link,
        reference_tool_link=reference_tool_link,
        candidate_tool_link=candidate_tool_link,
        arm_joints=arm_joints,
        tolerances=tolerances,
    )
    fk_results: tuple[FkSampleResult, ...] = ()
    endpoints_exist = all((
        reference_base_link in reference.links,
        candidate_base_link in candidate.links,
        reference_tool_link in reference.links,
        candidate_tool_link in candidate.links,
    ))
    required_joints_exist = all(
        joint in reference.joints and joint in candidate.joints
        for joint in arm_joints
    )
    topology_exists = not any(
        mismatch.category == 'chain_topology_mismatch'
        for mismatch in structural_mismatches
    )
    if (
        samples
        and endpoints_exist
        and required_joints_exist
        and topology_exists
    ):
        fk_results = compare_fk(
            reference,
            candidate,
            reference_base_link=reference_base_link,
            candidate_base_link=candidate_base_link,
            reference_tool_link=reference_tool_link,
            candidate_tool_link=candidate_tool_link,
            arm_joints=arm_joints,
            samples=samples,
            tolerances=tolerances,
        )
    return ParityResult(
        reference_source=reference.source,
        candidate_source=candidate.source,
        reference_base_link=reference_base_link,
        candidate_base_link=candidate_base_link,
        reference_tool_link=reference_tool_link,
        candidate_tool_link=candidate_tool_link,
        arm_joints=tuple(arm_joints),
        tolerances=tolerances,
        structural_mismatches=structural_mismatches,
        fk_results=fk_results,
    )


def compare_sources(
    reference_source: Path | str,
    candidate_source: Path | str,
    *,
    reference_xacro_args: Sequence[str] = (),
    candidate_xacro_args: Sequence[str] = (),
    **comparison_options: Any,
) -> ParityResult:
    """Load two URDF/xacro sources and compare their kinematic contracts."""
    reference = load_urdf_model(reference_source, reference_xacro_args)
    candidate = load_urdf_model(candidate_source, candidate_xacro_args)
    return compare_models(reference, candidate, **comparison_options)


def builtin_panda_samples(
    arm_joints: Sequence[str] = DEFAULT_ARM_JOINTS,
) -> tuple[JointSample, ...]:
    """Return deterministic, non-zero Panda configurations for CLI FK checks."""
    values = (
        (0.2, -0.5, 0.4, -1.5, 0.2, 1.2, -0.4),
        (0.5, -0.8, -0.3, -2.0, 0.7, 1.8, 0.6),
        (-0.4, 0.6, 0.8, -1.0, -0.5, 0.9, 1.0),
    )
    if len(arm_joints) != 7:
        raise ParitySetupError(
            'built-in FK samples require exactly seven configured arm joints; '
            'use --fk-sample or --no-fk'
        )
    return tuple(
        JointSample(
            name=f'sample_{index}',
            positions=dict(zip(arm_joints, positions)),
        )
        for index, positions in enumerate(values, start=1)
    )


def render_text(result: ParityResult) -> str:
    """Render a concise, actionable human-readable report."""
    lines = [
        f'Robot model parity: {"PASS" if result.passed else "FAIL"}',
        '',
        'Reference model:',
        f'  source: {result.reference_source}',
        f'  base: {result.reference_base_link}',
        f'  tool: {result.reference_tool_link}',
        '',
        'Candidate model:',
        f'  source: {result.candidate_source}',
        f'  base: {result.candidate_base_link}',
        f'  tool: {result.candidate_tool_link}',
        '',
        'Structural mismatches:',
    ]
    if result.structural_mismatches:
        for mismatch in result.structural_mismatches:
            lines.append(f'  - [{mismatch.category}] {mismatch.message}')
            if mismatch.reference is not None:
                lines.append(f'      reference: {mismatch.reference}')
            if mismatch.candidate is not None:
                lines.append(f'      candidate: {mismatch.candidate}')
            if mismatch.absolute_error is not None:
                lines.append(f'      absolute error: {mismatch.absolute_error}')
    else:
        lines.append('  - none')
    lines.extend(['', 'FK samples:'])
    if result.fk_results:
        for sample in result.fk_results:
            lines.extend([
                f'  - {sample.name}: {"PASS" if sample.passed else "FAIL"}',
                f'      reference_position: {list(sample.reference_position)}',
                f'      candidate_position: {list(sample.candidate_position)}',
                f'      position_error_m: {sample.position_error_m:.12g}',
                f'      orientation_error_rad: '
                f'{sample.orientation_error_rad:.12g}',
            ])
    else:
        lines.append('  - not requested or unavailable due to missing structure')
    lines.extend([
        '',
        'Summary:',
        f'  structural_mismatches: {len(result.structural_mismatches)}',
        f'  fk_mismatches: {result.fk_mismatch_count}',
        f'  total_mismatches: '
        f'{len(result.structural_mismatches) + result.fk_mismatch_count}',
    ])
    return '\n'.join(lines)


def render_json(result: ParityResult) -> str:
    """Render the stable machine-readable report."""
    return json.dumps(result.to_dict(), indent=2, sort_keys=True)


def resolve_current_panda_sources() -> tuple[Path, Path]:
    """Resolve the planning and Gazebo Panda sources from installed shares."""
    try:
        from ament_index_python.packages import (  # noqa: WPS433
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError as error:
        raise ParitySetupError(
            'ament_index_python is unavailable; source ROS 2 Jazzy first'
        ) from error
    try:
        reference_share = Path(get_package_share_directory(
            'moveit_resources_panda_moveit_config'
        ))
    except PackageNotFoundError as error:
        raise ParitySetupError(
            "required package 'moveit_resources_panda_moveit_config' is not "
            'installed; install the ROS 2 Jazzy MoveIt Panda resources'
        ) from error
    try:
        candidate_share = Path(get_package_share_directory(
            'adaptive_assembly_sim'
        ))
    except PackageNotFoundError as error:
        raise ParitySetupError(
            "package 'adaptive_assembly_sim' is not installed; build and "
            'source ~/ros2_adaptive_assembly_ws first'
        ) from error
    return (
        reference_share / 'config' / 'panda.urdf.xacro',
        candidate_share / 'urdf' / 'panda_gazebo_ros2_control.urdf.xacro',
    )


def _parse_sample(text: str) -> JointSample:
    if ':' not in text:
        raise ParitySetupError(
            f'invalid FK sample {text!r}; expected NAME:JOINT=VALUE,...'
        )
    name, assignments = text.split(':', 1)
    if not name or not assignments:
        raise ParitySetupError(
            f'invalid FK sample {text!r}; expected NAME:JOINT=VALUE,...'
        )
    positions: dict[str, float] = {}
    for assignment in assignments.split(','):
        if '=' not in assignment:
            raise ParitySetupError(
                f'invalid FK assignment {assignment!r} in sample {name!r}'
            )
        joint, value_text = assignment.split('=', 1)
        if not joint or joint in positions:
            raise ParitySetupError(
                f'invalid or duplicate joint {joint!r} in sample {name!r}'
            )
        try:
            positions[joint] = float(value_text)
        except ValueError as error:
            raise ParitySetupError(
                f'non-numeric value {value_text!r} for joint {joint!r}'
            ) from error
    return JointSample(name=name, positions=positions)


class _ArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise ParitySetupError(f'invalid arguments: {message}')


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = _ArgumentParser(
        description='Compare the kinematic parity of two URDF/xacro models.'
    )
    parser.add_argument('reference_source', nargs='?')
    parser.add_argument('candidate_source', nargs='?')
    parser.add_argument(
        '--current-panda-models', action='store_true',
        help='compare the installed MoveIt resources Panda and local sim model',
    )
    parser.add_argument('--base-link', default=DEFAULT_BASE_LINK)
    parser.add_argument('--reference-base-link')
    parser.add_argument('--candidate-base-link')
    parser.add_argument('--tool-link')
    parser.add_argument('--reference-tool-link')
    parser.add_argument('--candidate-tool-link')
    parser.add_argument(
        '--arm-joints', nargs='+', default=list(DEFAULT_ARM_JOINTS)
    )
    parser.add_argument('--reference-xacro-arg', action='append', default=[])
    parser.add_argument('--candidate-xacro-arg', action='append', default=[])
    parser.add_argument('--translation-tolerance', type=float, default=1e-6)
    parser.add_argument('--rotation-tolerance', type=float, default=1e-6)
    parser.add_argument('--axis-tolerance', type=float, default=1e-6)
    parser.add_argument('--joint-limit-tolerance', type=float, default=1e-6)
    parser.add_argument(
        '--fk-sample', action='append', default=[],
        help='repeatable NAME:JOINT=VALUE,... sample; replaces built-ins',
    )
    parser.add_argument('--no-fk', action='store_true')
    parser.add_argument('--json', action='store_true', dest='json_output')
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the command-line diagnostic and return its documented exit code."""
    parser = _build_argument_parser()
    json_requested = '--json' in (argv if argv is not None else sys.argv[1:])
    try:
        arguments = parser.parse_args(argv)
        if arguments.current_panda_models:
            if arguments.reference_source or arguments.candidate_source:
                raise ParitySetupError(
                    '--current-panda-models cannot be combined with sources'
                )
            reference_source, candidate_source = resolve_current_panda_sources()
        else:
            if not arguments.reference_source or not arguments.candidate_source:
                raise ParitySetupError(
                    'provide REFERENCE_SOURCE and CANDIDATE_SOURCE, or use '
                    '--current-panda-models'
                )
            reference_source = Path(arguments.reference_source)
            candidate_source = Path(arguments.candidate_source)

        reference_base = arguments.reference_base_link or arguments.base_link
        candidate_base = arguments.candidate_base_link or arguments.base_link
        reference_tool = (
            arguments.reference_tool_link
            or arguments.tool_link
            or DEFAULT_TOOL_LINK
        )
        candidate_tool = (
            arguments.candidate_tool_link
            or arguments.tool_link
            or (
                CURRENT_PANDA_TOOL_LINK
                if arguments.current_panda_models
                else DEFAULT_TOOL_LINK
            )
        )
        if arguments.no_fk and arguments.fk_sample:
            raise ParitySetupError('--no-fk cannot be combined with --fk-sample')
        if arguments.no_fk:
            samples: tuple[JointSample, ...] = ()
        elif arguments.fk_sample:
            samples = tuple(_parse_sample(value) for value in arguments.fk_sample)
        else:
            samples = builtin_panda_samples(arguments.arm_joints)
        tolerances = Tolerances(
            translation=arguments.translation_tolerance,
            rotation=arguments.rotation_tolerance,
            axis=arguments.axis_tolerance,
            joint_limit=arguments.joint_limit_tolerance,
        )
        result = compare_sources(
            reference_source,
            candidate_source,
            reference_xacro_args=arguments.reference_xacro_arg,
            candidate_xacro_args=arguments.candidate_xacro_arg,
            reference_base_link=reference_base,
            candidate_base_link=candidate_base,
            reference_tool_link=reference_tool,
            candidate_tool_link=candidate_tool,
            arm_joints=arguments.arm_joints,
            tolerances=tolerances,
            samples=samples,
        )
    except (ParitySetupError, OSError) as error:
        if json_requested:
            print(json.dumps({
                'schema_version': SCHEMA_VERSION,
                'error': str(error),
                'exit_code': 2,
            }, indent=2, sort_keys=True))
        else:
            print(f'Robot model parity setup error: {error}', file=sys.stderr)
        return 2

    print(render_json(result) if arguments.json_output else render_text(result))
    return 0 if result.passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
