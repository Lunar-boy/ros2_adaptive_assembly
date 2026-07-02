#!/usr/bin/env python3
"""Validate attach and detach without requiring Gazebo."""

from gazebo_attach_detach_test import run_check


if __name__ == '__main__':
    raise SystemExit(run_check('success'))
