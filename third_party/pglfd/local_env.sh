#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
export PYTHONPATH="$SCRIPT_DIR/build/pglfd/install/lib/python$PYTHON_VERSION/site-packages/:$SCRIPT_DIR/python:$PYTHONPATH"