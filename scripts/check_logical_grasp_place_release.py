#!/usr/bin/env python3
"""Validate release immediately after place-stage success."""

from logical_grasp_lifecycle_test import run_check


if __name__ == '__main__':
    raise SystemExit(run_check('place_release'))
