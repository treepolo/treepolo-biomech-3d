#!/bin/sh
# 重新產生肌肉／骨頭距離場資料並寫回 index.html（約 5 秒；需要 python3、numpy、scipy）
set -e
cd "$(dirname "$0")/.."
WD=$(mktemp -d)
python3 tools/extract_blobs.py index.html "$WD"
(cd "$WD" && python3 "$OLDPWD/tools/build.py")
python3 tools/inject.py index.html "$WD"
cat "$WD/build_report.json" | head -40
