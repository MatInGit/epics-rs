#!/usr/bin/env bash
# check-no-raw-tokio.sh — Verify that driver/example code uses the runtime
# facade (asyn_rs::runtime:: / epics_base_rs::runtime::) instead of tokio directly.
#
# Usage:
#   ./scripts/check-no-raw-tokio.sh          # check examples/
#   ./scripts/check-no-raw-tokio.sh path/    # check a custom directory
#
# Exit codes:
#   0  No direct tokio usage found
#   1  Direct tokio usage detected

set -euo pipefail

TARGET="${1:-examples}"
ERRORS=0

# --- Check .rs files for direct tokio:: usage ---
RS_HITS=$(grep -rn 'tokio::' --include='*.rs' "$TARGET" 2>/dev/null || true)
if [ -n "$RS_HITS" ]; then
    echo "ERROR: Direct tokio:: usage found in Rust sources."
    echo "       Use asyn_rs::runtime:: or epics_base_rs::runtime:: instead."
    echo ""
    echo "$RS_HITS"
    echo ""
    ERRORS=$((ERRORS + 1))
fi

# --- Check Cargo.toml for direct tokio dependency ---
TOML_HITS=$(grep -rn '^tokio\s*=' --include='Cargo.toml' "$TARGET" 2>/dev/null || true)
if [ -n "$TOML_HITS" ]; then
    echo "ERROR: Direct tokio dependency found in Cargo.toml."
    echo "       Depend on asyn-rs or epics-base-rs instead — they re-export async primitives."
    echo ""
    echo "$TOML_HITS"
    echo ""
    ERRORS=$((ERRORS + 1))
fi

if [ "$ERRORS" -gt 0 ]; then
    echo "FAILED: $ERRORS check(s) failed. See https://github.com/epics-rs/epics-rs#runtime-interface"
    exit 1
fi

echo "OK: No direct tokio usage in $TARGET"
