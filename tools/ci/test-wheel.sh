#!/usr/bin/env bash
set -eu
python -m pip install scikit-build-core numpy
python -m pip wheel --no-build-isolation --no-deps --wheel-dir build/wheels .
python -m pip install --no-deps --force-reinstall build/wheels/panthera_sdk-*.whl
python tests/test_python_sdk.py
python tests/test_python_examples.py
