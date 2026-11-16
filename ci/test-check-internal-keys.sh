#!/bin/sh
# The keys are built at run time so that this file holds no key itself.
set -eu

here=$(cd "$(dirname "$0")" && pwd)
check="$here/check-internal-keys.sh"
private_dir=${PRIVATE_PATH:-private}
key=$(printf '%s-%s' "S""V" 1234)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
failures=0

run_case() {
  name=$1 expected=$2 path=$3 content=$4
  repo="$work/$name"
  git init --quiet "$repo"
  mkdir -p "$repo/$private_dir" "$(dirname "$repo/$path")"
  echo "internal" > "$repo/$private_dir/keep.txt"
  printf '%s\n' "$content" > "$repo/$path"
  git -C "$repo" add -A
  git -C "$repo" -c user.name=t -c user.email=t@example.com commit --quiet -m fixture
  if (cd "$repo" && sh "$check" >/dev/null 2>&1); then actual=pass; else actual=fail; fi
  if [ "$actual" = "$expected" ]; then
    echo "ok   $name"
  else
    echo "FAIL $name: expected $expected, got $actual"
    failures=$((failures + 1))
  fi
}

run_case key-in-changelog fail CHANGELOG.md "- fix($key): a change"
run_case key-in-source fail tools/a.py "# see $key"
run_case key-only-in-private pass "$private_dir/notes.md" "see $key"
run_case no-key pass README.md "no ticket here"
run_case licence-version pass LICENSE "http://www.apache.org/licenses/LICENSE-2.0"
run_case key-inside-word pass README.md "XSV-12 is not a key"

[ "$failures" -eq 0 ]
