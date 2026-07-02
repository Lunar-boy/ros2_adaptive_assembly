#!/usr/bin/env python3
"""Validate deterministic behavior when the Gazebo service is absent."""

from gazebo_attach_detach_test import run_check


if __name__ == '__main__':
    raise SystemExit(run_check('failure'))
