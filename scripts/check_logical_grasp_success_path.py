#!/usr/bin/env python3
"""Validate the isolated logical grasp success lifecycle."""

import sys

from logical_grasp_lifecycle_test import run_check


if __name__ == '__main__':
    sys.exit(run_check('success'))
