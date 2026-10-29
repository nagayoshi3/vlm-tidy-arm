#!/bin/bash
# 複数の指示文で VLM の判断だけを取り、結果を1つの表にまとめる（アームは動かさない）。
# 使い方:  端末1で launch を起動した状態で
#   export TIDY_VLM_PROVIDER=openai; export OPENAI_API_KEY=...
#   bash scripts/compare_instructions.sh
# 結果は ~/tidy_logs/compare_<日時>.md に出力される。
set -e
OUT=~/tidy_logs/compare_$(date +%Y%m%d_%H%M%S).md
INSTRUCTIONS=(
  "これから資料作りをするので、仕事に集中できるように机を整理整頓して。"
  "お客さんが来るので、机の上をできるだけ空にして。"
  "コーヒーを飲みながら読書したい。読書に要らない物は片付けて。"
  "このボールペンはもう使わないので捨てていい。あとは資料作り用に整理整頓して。"
)
echo "# 指示文による判断の違い（$(date +%Y-%m-%d)）" > "$OUT"
for ins in "${INSTRUCTIONS[@]}"; do
  echo "=== $ins"
  before=$(ls -d ~/tidy_logs/2* 2>/dev/null | tail -1)
  ros2 run tidy_robot tidy_node --ros-args -p judge_only:=true -p start_delay:=1.0 -p instruction:="$ins" > /tmp/tidy_judge.log 2>&1 || true
  latest=$(ls -d ~/tidy_logs/2* | tail -1)
  if grep -q '"judge": "mock"' "$latest/plan.json"; then
    echo "!! VLM が使われず mock になりました。原因（ログの WARN 行）:"
    grep -E "WARN|ERROR" /tmp/tidy_judge.log | head -5
    echo "API キーを確認してから再実行してください。"; exit 1
  fi
  python3 - "$ins" "$latest/plan.json" >> "$OUT" <<'PY'
import json, sys
ins, path = sys.argv[1], sys.argv[2]
d = json.load(open(path, encoding='utf-8')); p = d['plan']
print(f"\n## 指示：{ins}\n\n- 判断：{d['judge']}（{d['elapsed_sec']} 秒）\n- 要約：{p['scene_summary']}\n- 順番の理由：{p['order_reason']}\n")
print("| 物体 | 判断 | 分類 | 理由 |\n| --- | --- | --- | --- |")
names = {'obj1':'ボールペン','obj2':'紙くず','obj3':'マグカップ','obj4':'空き缶','obj5':'消しゴム','obj6':'付箋'}
for it in p['items']:
    mark = {'keep': '片付ける', 'discard': '捨てる', 'leave': 'そのまま'}[it['decision']]
    print(f"| {names.get(it['id'], it['id'])} | {mark} | {it['category']} | {it['reason']} |")
PY
done
echo; echo "出力: $OUT"
