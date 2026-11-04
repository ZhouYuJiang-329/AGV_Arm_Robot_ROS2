"""Parse an optional map-frame AMCL initial pose supplied on the command line."""
import json
import math


def parse_initial_pose(value):
    """Return (x metres, y metres, yaw radians), or None for manual/default mode."""
    if not value.strip():
        return None
    message = 'initial_pose must be a finite numeric array [x, y, yaw], with yaw in radians'
    try:
        pose = json.loads(value)
        if not isinstance(pose, list) or len(pose) != 3:
            raise ValueError(message)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in pose):
            raise ValueError(message)
        pose = tuple(float(v) for v in pose)
        if not all(math.isfinite(v) for v in pose):
            raise ValueError(message)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError(message) from error
    return pose
