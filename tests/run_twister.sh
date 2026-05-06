#!/usr/bin/env bash
# Copyright (c) 2026 Freedom Veiculos Eletricos
# SPDX-License-Identifier: Apache-2.0
#
# Wrapper for Zephyr twister on native_sim: the NSI host runner defaults to -O0,
# which triggers glibc _FORTIFY_SOURCE warnings-as-errors without optimization.
#
# Prefer the repo devshell so Twister gets Python ≥3.11 and Zephyr deps (Twister
# imports datetime.UTC, jsonschema, etc.):
#   nix develop --accept-flake-config -c bash -lc 'export PATH="$(dirname "$(command -v python3)"):/usr/bin:/bin:$PATH"
#   export ZEPHYR_BASE="$PWD/deps/zephyr" ZEPHYR_EXTRA_MODULES="$PWD" && ./tests/run_twister.sh -T "$PWD/tests" -p native_sim/native/64'
# Put the devshell python3 first, then /usr/bin for cmake (avoids a broken
# ~/.local/bin/cmake and avoids /usr/bin/python3 3.10 shadowing the Nix 3.12).
#
# Build motor samples (NUCLEO-G474RE): same env, then e.g.:
#   ... && ./tests/run_twister.sh -T "$PWD/samples" -p nucleo_g474re --build-only
if command -v python3 >/dev/null 2>&1; then
	export PATH="$(dirname "$(command -v python3)"):/usr/bin:/bin:$PATH"
fi
export NSI_OPT=-O2
set -euo pipefail
ZEPHYR_BASE="${ZEPHYR_BASE:?set ZEPHYR_BASE to your Zephyr tree}"
exec "${ZEPHYR_BASE}/scripts/twister" "$@"
