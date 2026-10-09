#!/bin/bash
# Run kc_fit.py with its dependencies (embree ray tests, mesh simplification) in an isolated environment.
cd "$(dirname "$0")" && exec uv run --quiet --with trimesh --with embreex --with fast_simplification --with rtree --with scipy \
    --with opencv-python --with numpy --with pycollada --with networkx --with lxml python kc_fit.py "$@"
