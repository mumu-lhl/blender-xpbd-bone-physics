#!/usr/bin/env bash

set -u
set -o pipefail

report_dir="${1:-artifacts}"
mkdir -p "$report_dir"
report_dir="$(cd "$report_dir" && pwd)"
status=0

echo '$ cargo fmt --all --check' | tee "$report_dir/cargo-fmt.log"
cargo fmt --all --check 2>&1 | tee -a "$report_dir/cargo-fmt.log" || status=1

echo '$ cargo clippy --workspace --all-targets --features pyo3/extension-module -- -D warnings' \
  | tee "$report_dir/cargo-clippy.log"
cargo clippy --workspace --all-targets --features pyo3/extension-module -- -D warnings 2>&1 \
  | tee -a "$report_dir/cargo-clippy.log" || status=1

echo '$ cargo test -p bone_physics_core --all-targets' \
  | tee "$report_dir/cargo-core-test.log"
cargo test -p bone_physics_core --all-targets 2>&1 \
  | tee -a "$report_dir/cargo-core-test.log" || status=1

echo '$ cargo test -p bone_physics_py --test registry --test batch_validation --features pyo3/extension-module' \
  | tee "$report_dir/cargo-bridge-test.log"
cargo test -p bone_physics_py \
  --test registry \
  --test batch_validation \
  --features pyo3/extension-module 2>&1 \
  | tee -a "$report_dir/cargo-bridge-test.log" || status=1

echo '$ cargo bench -p bone_physics_core --no-run' | tee "$report_dir/cargo-bench-build.log"
cargo bench -p bone_physics_core --no-run 2>&1 \
  | tee -a "$report_dir/cargo-bench-build.log" || status=1

echo '$ cargo tree --duplicates' | tee "$report_dir/cargo-tree-duplicates.log"
cargo tree --duplicates 2>&1 | tee -a "$report_dir/cargo-tree-duplicates.log" || status=1

exit "$status"
