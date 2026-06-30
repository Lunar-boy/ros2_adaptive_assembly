# Adaptive Assembly Recovery

This package provides a message-only recovery supervisor for the adaptive
assembly planning and dry-run execution pipeline. It classifies status topics,
publishes deterministic recovery actions, and can optionally call the existing
PlanningScene reset services.

The supervisor never commands a robot, executes a trajectory, or retries a
plan. See `docs/recovery_supervisor.md` for its interfaces and validation.
