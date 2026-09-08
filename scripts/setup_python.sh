#!/usr/bin/env bash
set -Eeuo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
environment="${1:-$root/.venv}"

python3 -m venv "$environment"
"$environment/bin/python" -m pip install --upgrade pip
"$environment/bin/python" -m pip install -e "$root"

printf 'Activate with: source %q/bin/activate\n' "$environment"
