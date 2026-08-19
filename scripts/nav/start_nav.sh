#!/usr/bin/env bash
# [历史] 纯 Nav2 + MPPI — dev 已移除
set -euo pipefail
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
# shellcheck source=scripts/lib/nav_stack.sh
source "${ROOT}/scripts/lib/nav_stack.sh"
nav_stack_refused_msg
exit 2
