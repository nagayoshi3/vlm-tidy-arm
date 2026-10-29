# vlm-tidy-arm — AIが判断して机を整理整頓するCRANE+ V2

Physical AI 応用1講座 最終課題用のROS 2パッケージです。
散らかった机を真上のカメラで見て、**VLM（画像も読めるLLM）が「残す／捨てる」「どこにまとめるか」「片付ける順番」を判断**し、CRANE+ V2がその通りに片付けます。

- **整理**：指示の目的に照らして、要らない物（紙くず・空き缶など）をゴミ箱へ捨てる（discard）
- **整頓**：今は使わない物を分類ごとの置き場に、前から順に詰めて、向きを揃えて並べる（keep）。今まさに使う物は動かさず手元に残す（leave）

```
真上カメラ ──▶ 認識（物体ID・位置） ──▶ 整理判断（VLM） ──▶ 配置先決定（ルール） ──▶ 実行（逆運動学＋JointTrajectory）
  /tidy/camera/image      scene.yaml           vlm.py              planner.py              tidy_node.py
                                                  │
                                       判断と進行を /tidy/panel に表示（動画用）
```

意味の判断はAIに、座標の計算はルールに任せる、という役割分担がこの作品の設計方針です（VLMに座標まで出させると精度が出ないため）。

## 動作環境

講座の演習用Docker（`airobotbook/ros2-desktop-ai-robot-book-humble`）で、第4回・第5回と同じようにブラウザ（http://localhost:6080）から作業する前提です。

- ROS 2 Humble、Ignition Gazebo Fortress
- `~/airobot_ws` に `crane_plus`（AI-Robot-Book-Humble版）がビルド済み（イメージに最初から入っています）
- VLMを使う場合はAPIキーが1つ（Anthropic / OpenAI / Gemini のどれか）。**キーがなくても mock モードで最後まで動きます**

## インストール

コンテナ内の端末で実行します。

```bash
cd ~/pai_ws/src
# GitHub に上げた場合は git clone、手元の zip なら /workspace（第5回の -v マウント先）経由でコピー
cp -r /workspace/tidy_robot .
cd ~/pai_ws
colcon build --symlink-install --packages-select tidy_robot
source install/setup.bash
```

オフラインテスト（ROSなしで、手が届くか・計画・VLM出力の検証・SDFを確認）：

```bash
cd ~/pai_ws/src/tidy_robot && python3 -m pytest -q test/test_offline.py
```

## 実行（端末を3つ使う）

**端末1：シミュレータ**（ワールドは `config/scene.yaml` から自動生成）

```bash
ros2 launch tidy_robot tidy_sim.launch.py
```

**端末2：AIの視点パネル**（カメラ画像に判断結果と進行状況を重ねて表示）

```bash
ros2 run rqt_image_view rqt_image_view /tidy/panel/compressed
```

**端末3：整理整頓ノード**

```bash
# まずは mock（APIキー不要、scene.yaml に書いた正解で動く）で動作確認
ros2 run tidy_robot tidy_node

# VLM を使う（例：Anthropic。openai / gemini も同様）
export TIDY_VLM_PROVIDER=anthropic
export ANTHROPIC_API_KEY=sk-...        # OPENAI_API_KEY / GEMINI_API_KEY
ros2 run tidy_robot tidy_node

# 指示を変えると判断が変わる（動画で2パターン見せると、AIを使う意味が伝わります）
ros2 run tidy_robot tidy_node --ros-args -p instruction:="コーヒーを飲みながら作業したい。必要な物だけ残して片付けて。"
```

主なパラメータ：

| パラメータ | 既定値 | 内容 |
|---|---|---|
| `instruction` | scene.yaml の値 | ユーザーの指示文 |
| `provider` / `model` | 環境変数 | VLM の切り替え（`TIDY_VLM_PROVIDER` / `TIDY_VLM_MODEL` でも可） |
| `speed` | 1.0 | 動作速度の倍率（重い環境では 0.5 など） |
| `start_delay` | 3.0 | カメラ画像が届いてから判断を始めるまでの秒数（録画開始の余裕） |
| `judge_only` | false | true なら VLM の判断だけ行い、アームは動かさない |

launch 側の引数：`position_gain`（既定 0.5）。Gazebo の関節位置制御の P ゲインで、既定の 0.1 だとアームが目標の手前で止まりやすいため上げている。ふらつく場合は `position_gain:=0.3` のように下げる。

VLM への入力画像・プロンプト・生の応答・計画（JSON）は、実行のたびに `~/tidy_logs/日時/` に保存されます。レポートに貼る材料になります。

## 初級版（物体3個・置き場1種類）で動かす

まず小さく動かしたいときは `config/scene_lite.yaml` を使います（ペン・紙くず・マグカップの3個、置き場1種類＋ゴミ箱）。カメラ・VLM・手先追従はそのままです。

```bash
LITE=$(ros2 pkg prefix tidy_robot)/share/tidy_robot/config/scene_lite.yaml
ros2 launch tidy_robot tidy_sim.launch.py scene_file:=$LITE          # 端末1
ros2 run tidy_robot tidy_node --ros-args -p scene_file:=$LITE        # 端末3
```

動いたら `scene.yaml`（6個・分類3種類）に戻すと、整頓の見せ場が増えます。

## 指示文を変えて判断の違いを見る（動画の見せ場）

同じ机・同じ物でも、指示が変われば VLM の判断が変わります。アームを動かさず判断だけを取るモードがあるので、短時間で比較できます。

```bash
# 判断だけ（アームは動かない、1回10秒程度）
ros2 run tidy_robot tidy_node --ros-args -p judge_only:=true -p instruction:="お客さんが来るので、机の上をできるだけ空にして。"

# 4つの指示文をまとめて比較し、Markdown の表にする
bash $(ros2 pkg prefix tidy_robot)/share/tidy_robot/scripts/compare_instructions.sh
```

結果は `~/tidy_logs/compare_日時.md` に出ます。レポートの表にそのまま貼れます。指示文は `scripts/compare_instructions.sh` の `INSTRUCTIONS` で変えられます。

動画では、既定の指示で1本通したあと、別の指示（例：「このボールペンは捨てていい」）でもう1本撮り、判断が変わる場面を並べると伝わりやすいです。

## 机の上を変える

`config/scene.yaml` を編集します（物体の形・色・位置、置き場、ゴミ箱、カメラ、既定の指示）。位置はロボット土台からの距離 `r` と角度 `theta_deg` で指定します。CRANE+ V2 が真上から掴めるのは `r` が約 0.05〜0.16 m の範囲です。編集したら `python3 -m pytest -q test/test_offline.py` を実行すると、届かない位置や重なりを検出できます。

## 動画の撮り方（1〜3分）

1. Gazebo、rqt_image_view（`/tidy/panel`）、端末3 が並んで見えるように、VNC 画面内でウィンドウを配置する。Gazebo の視点は launch ファイルの `GUI_CAMERA_POSE` で調整できます
2. Mac の画面収録（`Cmd + Shift + 5` →「選択部分を収録」）でブラウザの VNC 部分を囲み、録画を開始する
3. 端末3 で `tidy_node` を起動する（`start_delay` の3秒後に AI の判断が始まる）
4. 「整理整頓 完了！」が出たら録画を停止する
5. 編集で、冒頭に「タイトル＋指示文」、途中に「AI の判断結果（パネル）」のアップ、最後に「before / after」を入れると伝わりやすい。長い場合は 2 倍速にして、**動画内か概要欄に「2倍速」と明記**する

構成例（2倍速で約2分）：タイトル（5秒）→ 散らかった机（5秒）→ AI が判断しパネルに結果が出る（15秒）→ ゴミを捨てる → 文房具・食器を揃えて並べる → 完了・before/after（10秒）→ 指示を変えた別パターン（任意）

## 設計上の割り切り（レポートに明記してください）

- **把持は「attach 方式」**：Gazebo の物理把持は滑りやすく不安定なため、机の上の物は重力・衝突なしの小物として置き、グリッパを閉じたら Gazebo の `set_pose` サービスで手先に追従させています（`gz_pose.py`）。アームの動作（逆運動学、関節軌道）は本物です
- **物体認識はシーン定義を使用**：物体の ID・名前・位置は `scene.yaml` から取っています（シミュレータの正解座標）。VLM には実際のカメラ画像も渡しており、判断（整理・整頓・順番）は VLM が行います。オープン語彙検出器（OWL-ViT など）への置き換えは今後の課題です
- **VLM の出力は検証してから使う**：JSON の形式、全物体が含まれているか、分類名が正しいかを確認し、置き場が足りなければ空きのある場所に回します。2回失敗したら mock の判断で続行します（`vlm.py`）

## うまく動かないとき

- **ノードが「ロボットの起動を待っています」から進まない**：`ros2 node list` に `/controller_manager` が無ければ Gazebo が落ちているので launch を再起動。出ているのに進まない場合は、同じマシンの ROS プロセス数が多いと通信層（CycloneDDS）が新しいプロセスを見つけられないことがある。`~/.bashrc` に次を追加してコンテナを再起動する：
  `export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'`
  診断には `python3 scripts/sub_test.py 1`（購読が届くかを 5 秒ごとに表示）が使える。tidy_node 自体も 10 秒ごとに受信回数を表示する
- **ノードを止めるときは Ctrl+C**。Ctrl+Z（一時停止）や `kill -9` は残骸が通信を乱すので避ける

- **カメラ画像が来ない**：`ros2 topic hz /tidy/camera/image` で確認してください。来ない場合でも、30秒待ってから画像なしで VLM に判断させて続行します。パネルは灰色の背景になります
- **物体が手先についてこない**：`ign service -l | grep set_pose` で `/world/default/set_pose` があるか確認してください。追従が飛び飛びに見える場合は、`speed:=0.5` で動作を遅くすると目立たなくなります
- **アームが目標に届かない警告が出る**：シミュレーションが実時間より遅いと出ることがありますが、そのまま続行します。気になる場合は `speed` を下げてください
- **API キーのエラー**：キーが未設定、またはエラーが続く場合は自動的に mock で続行します。端末の警告を確認してください

## セキュリティ

API キーは**環境変数で渡し、コードや GitHub に絶対に書かないでください**（提出リポジトリは public です）。`.gitignore` で `.env` とログを除外しています。また、提出物に氏名・メールアドレスを入れない決まりがあるので、`package.xml` と `setup.py` の maintainer は omnicampus のアカウント名に書き換えてください。

## ライセンス

Apache-2.0。以下の Apache-2.0 のコードをもとにしています。

- `tidy_robot/kinematics.py`：『ROS 2とPythonで作って学ぶAIロボット入門 改訂第2版』chapter6 `crane_plus_commander/kinematics.py`（AI-Robot-Book-Humble）
- `launch/tidy_sim.launch.py`：`crane_plus_gazebo/no_moveit_crane_plus_with_table.launch.py`（RT Corporation）
