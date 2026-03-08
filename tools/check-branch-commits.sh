#!/usr/bin/env bash
# Check commit messages on the current branch against its upstream base.
# Usage: tools/check-branch-commits.sh [base_ref]
#   base_ref: defaults to master

set -euo pipefail

base="${1:-master}"
head="HEAD"

# Find merge base to only check branch-specific commits
merge_base=$(git merge-base "$base" "$head" 2>/dev/null) || {
    echo "Error: cannot find merge base between '$base' and '$head'" >&2
    exit 1
}

commits=$(git rev-list "$merge_base..$head")

if [ -z "$commits" ]; then
    echo "No commits to check (branch is up-to-date with $base)."
    exit 0
fi

errors=0

for sha in $commits; do
    subject=$(git log --format='%s' -1 "$sha")
    short=$(git rev-parse --short "$sha")

    # Empty subject
    if [ -z "$subject" ]; then
        echo "ERROR: $short has empty commit message"
        errors=$((errors + 1))
        continue
    fi

    # Fixup/squash commits
    if echo "$subject" | grep -qE '^(fixup|squash)! '; then
        echo "ERROR: $short is a fixup/squash commit: $subject"
        errors=$((errors + 1))
    fi

    # WIP commits
    if echo "$subject" | grep -qiE '^WIP'; then
        echo "ERROR: $short is a WIP commit: $subject"
        errors=$((errors + 1))
    fi

    # Subject too long (72 chars conventional limit)
    if [ ${#subject} -gt 72 ]; then
        echo "WARNING: $short subject exceeds 72 chars (${#subject}): $subject"
    fi
done

count=$(echo "$commits" | wc -w)

if [ "$errors" -gt 0 ]; then
    echo ""
    echo "FAILED: $errors error(s) found in $count commit(s)."
    exit 1
fi

echo "OK: $count commit(s) checked, no issues found."
