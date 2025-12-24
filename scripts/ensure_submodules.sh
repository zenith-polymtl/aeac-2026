#!/usr/bin/env bash
set -e
git submodule update --init --recursive
# Optional: move submodules to latest of their tracked branches
git submodule update --remote --merge
