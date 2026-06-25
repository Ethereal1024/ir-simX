#!/usr/bin/env bash
# Switch to original ir-sim (ML environment) for testing.
# Uses Python's -P flag to prevent CWD from being prepended to sys.path,
# so the installed irsim package takes priority over local source.
#
# Usage:
#   ./run_original.sh -m pytest tests/test_kinematics.py -v
#   ./run_original.sh usage/17gui_world/gui.py
set -e
export PYTHONPATH=""
exec conda run -n ML python -P "$@"
