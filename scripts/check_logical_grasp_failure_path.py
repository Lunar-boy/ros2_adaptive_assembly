#!/usr/bin/env python3
"""Validate deterministic logical grasp failure handling."""

import sys

from logical_grasp_lifecycle_test import run_check


if __name__ == '__main__':
    sys.exit(run_check('failure'))
