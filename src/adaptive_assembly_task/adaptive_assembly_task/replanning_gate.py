"""Decision logic for filtering small target-pose movements."""


def should_replan(distance: float, threshold: float) -> bool:
    """Return whether a target update should produce new planning targets."""
    return threshold <= 0.0 or distance > threshold
