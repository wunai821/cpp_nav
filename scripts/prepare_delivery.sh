#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DELIVERY_ROOT="${1:-${ROOT_DIR}/delivery/rm_sentinel_nav_clean}"

rm -rf "${DELIVERY_ROOT}"
mkdir -p "${DELIVERY_ROOT}"

copy_path() {
  local path="$1"
  if [[ -e "${ROOT_DIR}/${path}" ]]; then
    cp -R "${ROOT_DIR}/${path}" "${DELIVERY_ROOT}/"
  fi
}

copy_config_dir() {
  mkdir -p "${DELIVERY_ROOT}/config"
  find "${ROOT_DIR}/config" -maxdepth 1 -type f -exec cp {} "${DELIVERY_ROOT}/config/" \;
}

# Minimal delivery bundle: source, configs, docs, tests, and root build metadata.
copy_path "src"
copy_path "include"
copy_config_dir
copy_path "docs"
copy_path "tests"
copy_path "CMakeLists.txt"
copy_path "README.md"
copy_path "LICENSE"
copy_path ".gitignore"

printf 'Created clean delivery bundle at %s\n' "${DELIVERY_ROOT}"
