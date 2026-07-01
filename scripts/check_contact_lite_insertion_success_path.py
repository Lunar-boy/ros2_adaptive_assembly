#!/usr/bin/env python3
"""Validate contact-lite insertion success with synthetic poses."""

import sys

from contact_lite_insertion_test import run_check


if __name__ == '__main__':
    sys.exit(run_check('success'))
