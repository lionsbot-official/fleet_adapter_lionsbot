#!/bin/sh
# Reads the committed tree (HEAD), not the working tree.
set -eu

prefixes=${INTERNAL_KEY_PREFIXES:-SV}
private=${PRIVATE_PATH:-private}
alternatives=$(printf '%s' "$prefixes" | tr ' ,' '||')
pattern="(^|[^A-Za-z0-9])($alternatives)-[0-9]+"

found=$(git grep -InE "$pattern" HEAD -- ":!${private}/" || true)
if [ -n "$found" ]; then
  echo "ERROR: published files hold internal ticket keys:"
  echo "$found"
  echo "       Name the change in plain words, or by its public release commit."
  exit 1
fi
echo "No internal ticket key in published files."
