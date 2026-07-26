#!/usr/bin/env bash

set -u
set -o pipefail

report_dir="${1:-artifacts}"
mkdir -p "$report_dir"
status=0

echo '$ cargo fmt --all --check' | tee "$report_dir/cargo-fmt.log"
cargo fmt --all --check 2>&1 | tee -a "$report_dir/cargo-fmt.log" || status=1

echo '$ cargo clippy --workspace --all-targets -- -D warnings' \
  | tee "$report_dir/cargo-clippy.log"
cargo clippy --workspace --all-targets -- -D warnings 2>&1 \
  | tee -a "$report_dir/cargo-clippy.log" || status=1

echo '$ cargo test --workspace --all-targets' \
  | tee "$report_dir/cargo-test.log"
cargo test --workspace --all-targets 2>&1 \
  | tee -a "$report_dir/cargo-test.log" || status=1

echo '$ cargo tree --duplicates' | tee "$report_dir/cargo-tree-duplicates.log"
cargo tree --duplicates 2>&1 | tee -a "$report_dir/cargo-tree-duplicates.log" || status=1

exit "$status"
