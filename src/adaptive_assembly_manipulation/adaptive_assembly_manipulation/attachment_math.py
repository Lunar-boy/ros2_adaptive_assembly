"""Small dependency-free helpers for kinematic Gazebo attachment."""


def rotate_vector_by_quaternion(vector, quaternion):
    """Rotate a three-component vector by an (x, y, z, w) quaternion."""
    vx, vy, vz = vector
    qx, qy, qz, qw = quaternion
    norm_squared = qx * qx + qy * qy + qz * qz + qw * qw
    if norm_squared == 0.0:
        raise ValueError('quaternion must have non-zero norm')

    # q * (v, 0) * inverse(q), expanded to avoid external dependencies.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    scale = 1.0 / norm_squared
    return (
        vx + scale * (qw * tx + qy * tz - qz * ty),
        vy + scale * (qw * ty + qz * tx - qx * tz),
        vz + scale * (qw * tz + qx * ty - qy * tx),
    )
