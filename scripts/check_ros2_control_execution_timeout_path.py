#!/usr/bin/env python3
"""Validate deterministic result timeout and cancellation handling."""

import sys

from ros2_control_execution_path_test import run_check


if __name__ == '__main__':
    sys.exit(run_check('timeout'))
