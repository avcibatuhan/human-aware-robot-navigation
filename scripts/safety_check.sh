#!/usr/bin/env bash
# Repository safety check. Runs before every push (.githooks/pre-push) and in CI.
#
#   scripts/safety_check.sh [<commit-range>]
#
# Checks the committed tree at HEAD and, if a range is given, every file added
# by the commits in that range (so a forbidden file that was added and later
# deleted is still caught before it reaches the remote).
set -uo pipefail

cd "$(git rev-parse --show-toplevel)"
RANGE="${1:-}"
MAX_BYTES=$((1024 * 1024))            # 1 MB for ordinary files
MAX_GIF_BYTES=$((10 * 1024 * 1024))   # docs/demo.gif only
fail=0

err() {
  echo "  ✗ $*" >&2
  fail=1
}

# Files that must never be committed: weights, bags, videos, build output,
# the dissertation, agent instructions, credentials.
FORBIDDEN='(\.(pt|onnx|engine|mcap|db3|pdf|mp4|avi|mkv|webm|mov|pem|key)$)|(^|/)(build|install|log|bags)/|(^|/)CLAUDE\.md$|(^|/)\.env(\..*)?$|(^|/)id_(rsa|ed25519)$|(^|/)\.claude/'

# Credential patterns.
SECRETS='gh[pousr]_[A-Za-z0-9]{36}|github_pat_[A-Za-z0-9_]{40,}|sk-ant-[A-Za-z0-9_-]{20,}|AKIA[0-9A-Z]{16}|hf_[A-Za-z0-9]{30,}|-----BEGIN [A-Z ]*PRIVATE KEY-----'

echo "safety check: tree at $(git rev-parse --short HEAD)${RANGE:+, commits $RANGE}"

names="$(git ls-tree -r --name-only HEAD)"
if [[ -n "$RANGE" ]]; then
  names+=$'\n'"$(git log --diff-filter=A --name-only --format= "$RANGE")"
fi
while IFS= read -r f; do
  [[ -z "$f" ]] && continue
  if [[ "$f" =~ $FORBIDDEN ]]; then
    err "forbidden file: $f"
  fi
done < <(sort -u <<<"$names")

while read -r _mode _type _sha size path; do
  limit=$MAX_BYTES
  [[ "$path" == "docs/demo.gif" ]] && limit=$MAX_GIF_BYTES
  if ((size > limit)); then
    err "file too large: $path ($((size / 1024)) kB, limit $((limit / 1024)) kB)"
  fi
done < <(git ls-tree -r -l HEAD)

if hits="$(git grep -nIE -e "$SECRETS" HEAD -- . 2>/dev/null)"; then
  err "possible credential in tracked files:"
  cut -d: -f2,3 <<<"$hits" | sed 's/^/      /' >&2
fi
if [[ -n "$RANGE" ]] && git log -p --format= "$RANGE" | grep -qE -e "^\+.*($SECRETS)"; then
  err "possible credential added in commits $RANGE"
fi

if command -v ruff >/dev/null 2>&1; then
  ruff check --quiet . || err "ruff check failed"
else
  echo "  - ruff not installed here, lint skipped (CI runs it)"
fi

if ((fail)); then
  echo "safety check FAILED" >&2
  exit 1
fi
echo "safety check passed"
