#!/usr/bin/env bash
set -euo pipefail

# 1) Init/update at the exact commits pinned in the superproject
git submodule sync --recursive
git submodule update --init --recursive

# 2) Ensure "submodule.<name>.branch" is set in .git/config
#    so `git submodule update --remote` doesn't default to origin/master.
while read -r key branch; do
  name="${key#submodule.}"
  name="${name%.branch}"
  git config "submodule.${name}.branch" "$branch"
done < <(git config -f .gitmodules --get-regexp '^submodule\..*\.branch$' || true)

# 3) Optional: move submodules to latest of their tracked branches
git submodule update --remote --merge --recursive
