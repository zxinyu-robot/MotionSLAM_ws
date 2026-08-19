#!/usr/bin/env bash
# [历史] NavFn + ScanFollowPath Hybrid — 仅在 feature/hybrid-baseline 维护
set -euo pipefail
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
# shellcheck source=scripts/lib/nav_stack.sh
source "${ROOT}/scripts/lib/nav_stack.sh"
nav_stack_refused_msg
exit 2
