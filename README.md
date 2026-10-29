# vlm-tidy-arm — VLM が判断して机を整理整頓するロボットアーム

散らかった机を真上のカメラで見て、**VLM（画像も読める大規模言語モデル）が「捨てる／片付ける／そのまま」「どこにまとめるか」「片付ける順番」を判断**し、小型ロボットアーム CRANE+ V2 がシミュレータ（Gazebo）上でその通りに片付けます。

同じ机・同じ物でも、「資料作りに集中したい」「コーヒーを飲みながら読書したい」「このペンは捨てていい」と指示が変われば片付け方が変わる、というところが見どころです。

- **整理**：指示の目的に照らして要らない物（紙くず・空き缶など）をゴミ箱へ捨てる
- **整頓**：今は使わない物を分類ごとの置き場に、前から順に詰めて向きを揃えて並べる。今まさに使う物（読書中のマグカップなど）は動かさず手元に残す

動画（2 倍速）：（YouTube の URL: https://youtu.be/rtzc8rdOenA?si=iTYu5fL9vR1jsb_f）

## 仕組み

```
真上カメラ ──▶ 認識（物体の名前と位置） ──▶ 判断（VLM） ──▶ 配置先決定（ルール） ──▶ 実行（逆運動学＋関節軌道）
  Gazebo のカメラ       config/scene.yaml        vlm.py             planner.py              tidy_node.py
                                                   │
                                    判断の結果と進行状況をカメラ画像に重ねて表示（view.py）
```

設計方針は「**意味の判断は VLM に、座標の計算はルールに**」です。

- VLM には、真上カメラの画像、物体のリスト、片付け先の一覧、ユーザーの指示文を渡し、各物体について `discard`（捨てる）/ `keep`（片付ける）/ `leave`（使うので動かさない）、分類、片付ける順番、理由を JSON で返させます
- VLM に座標は出させません。置き場のどの枠に、どの向きで置くかはルール（`planner.py`）が決め、関節角は逆運動学（`kinematics.py`）で計算します。座標は VLM の不得意分野で、ルールに任せたほうが動作が確実だからです
- VLM の出力は形式・物体の抜け・分類名を検証してから使い、2 回失敗したら固定ルールの判断で続行します

### コードの構成

| ファイル | 役割 |
|---|---|
| `tidy_robot/tidy_node.py` | メインノード。起動・VLM への問い合わせ・アーム動作・物体の追従・パネル表示 |
| `tidy_robot/vlm.py` | プロンプト生成、VLM API 呼び出し（Anthropic / OpenAI / Gemini）、出力の検証とフォールバック |
| `tidy_robot/planner.py` | VLM の判断を置き場の座標と向きに変換。手が届くかの事前検証 |
| `tidy_robot/kinematics.py` | CRANE+ V2 の順運動学・逆運動学 |
| `tidy_robot/scene.py`, `world_gen.py` | `scene.yaml` から Gazebo のワールド（SDF）を生成 |
| `tidy_robot/gz_pose.py` | Gazebo の `set_pose` サービスで物体を動かす |
| `tidy_robot/view.py` | 判断と進行状況のパネル描画、VLM に渡す画像の注釈 |
| `launch/tidy_sim.launch.py` | Gazebo・ロボット・コントローラ・カメラ中継の一括起動 |
| `config/scene.yaml` | 物体・置き場・ゴミ箱・カメラ・既定の指示文の定義 |
| `scripts/compare_instructions.sh` | 複数の指示文で VLM の判断だけを取り、表にまとめる |
| `test/test_offline.py` | ROS なしで、到達性・計画・VLM 出力の検証・SDF 生成をテスト |

## 動作環境

- ROS 2 Humble、Ignition Gazebo Fortress
- ロボット：CRANE+ V2（RT Corporation）。モデルとコントローラは [AI-Robot-Book-Humble](https://github.com/AI-Robot-Book-Humble) 版の `crane_plus` パッケージを使用
- VLM：OpenAI gpt-4o-mini で確認。Anthropic / Gemini にも切り替え可。**API キーがなくても mock モード（設定ファイルに書いた正解で動く）で最後まで動きます**

上記がまとめて入った公開 Docker イメージ `airobotbook/ros2-desktop-ai-robot-book-humble`（書籍『ROS 2とPythonで作って学ぶAIロボット入門 改訂第2版』の演習用）を使うのが簡単です。ブラウザからデスクトップ環境を操作できます。

```bash
# ホスト側。<作業フォルダ> はこのリポジトリを置いた場所
docker run -d --name pai_ros2 -p 6080:80 --shm-size=512m \
  -v "<作業フォルダ>":/workspace \
  airobotbook/ros2-desktop-ai-robot-book-humble:latest
# ブラウザで http://localhost:6080 を開く
```

## インストール

コンテナ内の端末で実行します（`~/airobot_ws` に `crane_plus` がビルド済みの前提。上のイメージなら最初から入っています）。

```bash
mkdir -p ~/pai_ws/src && cd ~/pai_ws/src
git clone https://github.com/nagayoshi3/vlm-tidy-arm.git tidy_robot
cd ~/pai_ws
source ~/airobot_ws/install/setup.bash
colcon build --symlink-install --packages-select tidy_robot
source install/setup.bash
```

ROS を起動せずに確認できるオフラインテスト：

```bash
cd ~/pai_ws/src/tidy_robot && python3 -m pytest -q test/test_offline.py
```

## 実行

端末を 3 つ使います。各端末で `source ~/pai_ws/install/setup.bash` を済ませておきます。

**端末 1：シミュレータ**（ワールドは `config/scene.yaml` から自動生成されます）

```bash
ros2 launch tidy_robot tidy_sim.launch.py
```

**端末 2：整理整頓ノード**

```bash
# まず mock（API キー不要）で動作確認
ros2 run tidy_robot tidy_node

# VLM を使う（例：OpenAI。anthropic / gemini も同様）
export TIDY_VLM_PROVIDER=openai
export OPENAI_API_KEY=sk-...          # ANTHROPIC_API_KEY / GEMINI_API_KEY
ros2 run tidy_robot tidy_node

# 指示文を変える
ros2 run tidy_robot tidy_node --ros-args -p instruction:="コーヒーを飲みながら読書したい。読書に要らない物は片付けて。"
```

**端末 3：AI の視点パネル**（カメラ画像に判断結果と進行状況を重ねたもの。ノード起動後に開きます）

```bash
ros2 run rqt_image_view rqt_image_view /tidy/panel --ros-args -p image_transport:=compressed
```

主なパラメータ：

| パラメータ | 既定値 | 内容 |
|---|---|---|
| `instruction` | scene.yaml の値 | ユーザーの指示文 |
| `provider` / `model` | 環境変数 | VLM の切り替え（`TIDY_VLM_PROVIDER` / `TIDY_VLM_MODEL` でも可） |
| `speed` | 1.0 | 動作速度の倍率（重い環境では 0.5 など） |
| `start_delay` | 3.0 | カメラ画像が届いてから判断を始めるまでの秒数 |
| `judge_only` | false | true なら VLM の判断だけ行い、アームは動かさない |
| `panel_scale` | 1.0 | 無圧縮パネル画像の縮小率（通信が詰まるときは 0.5） |

launch の引数 `position_gain`（既定 0.5）は Gazebo の関節位置制御の P ゲインです。既定の 0.1 だとアームが目標の手前で止まりやすいため上げています。ふらつく場合は `position_gain:=0.3` のように下げてください。

VLM への入力画像・プロンプト・生の応答・計画（JSON）は、実行のたびに `~/tidy_logs/日時/` に保存されます。

## 指示文を変えて判断の違いを見る

アームを動かさず判断だけを取るモードがあり、短時間で比較できます。

```bash
# 判断だけ（1 回 10 秒程度）
ros2 run tidy_robot tidy_node --ros-args -p judge_only:=true -p instruction:="お客さんが来るので、机の上をできるだけ空にして。"

# 4 つの指示文をまとめて比較し、Markdown の表にする
bash $(ros2 pkg prefix tidy_robot)/share/tidy_robot/scripts/compare_instructions.sh
```

結果は `~/tidy_logs/compare_日時.md` に出ます。指示文は `scripts/compare_instructions.sh` の `INSTRUCTIONS` で変えられます。gpt-4o-mini での結果の例：

| 指示 | ボールペン | 紙くず | マグカップ | 空き缶 | 消しゴム | 付箋 |
|---|---|---|---|---|---|---|
| 資料作りに集中したい | 片付ける | 捨てる | 片付ける | 捨てる | 片付ける | 片付ける |
| お客さんが来るので机を空にして | 片付ける | 捨てる | 片付ける | 捨てる | 片付ける | 片付ける |
| コーヒーを飲みながら読書したい | 片付ける | 捨てる | **そのまま** | 捨てる | 片付ける | 片付ける |
| このボールペンは捨てていい | **捨てる** | 捨てる | 片付ける | 捨てる | 片付ける | 片付ける |

## 机の上を変える

`config/scene.yaml` を編集します（物体の形・色・位置、置き場、ゴミ箱、カメラ、既定の指示文）。位置はロボット土台からの距離 `r` と角度 `theta_deg` で指定します。CRANE+ V2 が真上から掴めるのは `r` が約 0.05〜0.16 m の範囲です。編集後に `python3 -m pytest -q test/test_offline.py` を実行すると、届かない位置や重なりを検出できます。

小さく試したいときは `config/scene_lite.yaml`（物体 3 個・置き場 1 種類）があります：

```bash
LITE=$(ros2 pkg prefix tidy_robot)/share/tidy_robot/config/scene_lite.yaml
ros2 launch tidy_robot tidy_sim.launch.py scene_file:=$LITE
ros2 run tidy_robot tidy_node --ros-args -p scene_file:=$LITE
```

## 設計上の割り切り

- **把持は物理的には掴んでいません**：Gazebo での小物の把持は滑りやすく不安定なため、机の上の物は物理計算の対象外（static）として置き、グリッパを閉じたら Gazebo の `set_pose` サービスで物体を手先に追従させています（`gz_pose.py`）。アームの動作（逆運動学、関節軌道）は実際に実行しています
- **物体の位置と置き場の位置はシーン定義で与えています**：物体の名前・位置、置き場とゴミ箱の位置は `scene.yaml` から取ります。VLM には実際のカメラ画像も渡しますが、位置の推定は任せず、判断（捨てる／片付ける／そのまま、分類、順番）だけを VLM が行います。画像からの物体認識（オープン語彙検出など）は今後の課題です
- **追従の遅延**：`set_pose` はコマンド経由で 1 回 0.1〜0.3 秒かかります。関節速度から先回りさせ、残る遅れは動作速度を落として目立たなくしています

## うまく動かないとき

- **ノードが「ロボットの起動を待っています」から進まない**：`ros2 node list` に `/controller_manager` が無ければ Gazebo が落ちています。launch を再起動してください。出ているのに進まない場合は、同じマシンの ROS プロセスが多いと通信層（CycloneDDS）が新しいプロセスを見つけられないことがあります。`~/.bashrc` に次を追加してコンテナを再起動してください：
  `export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'`
  診断には `python3 scripts/sub_test.py 1`（購読が届くかを 5 秒ごとに表示）が使えます。`tidy_node` 自体も 10 秒ごとに受信回数を表示します
- **パネルが途中で止まる**：無圧縮画像（1.4 MB/枚）の配信が重い環境で詰まることがあります。`/tidy/panel/compressed`（JPEG）を使ってください。表示されない場合は `sudo apt install ros-humble-compressed-image-transport`
- **ノードを止めるときは Ctrl+C**。Ctrl+Z や `kill -9` は残骸が通信を乱すので避けてください
- **カメラ画像が来ない**：`ros2 topic hz /tidy/camera/image` で確認してください。来ない場合でも 30 秒待ってから画像なしで VLM に判断させて続行します
- **物体が手先についてこない**：`ign service -l | grep set_pose` で `/world/default/set_pose` があるか確認してください。追従が飛び飛びに見える場合は `speed:=0.5` で動作を遅くすると目立たなくなります
- **アームが目標に届かない警告が出る**：シミュレーションが実時間より遅いと出ることがありますが、そのまま続行します。`position_gain` や `speed` を調整してください
- **判断が mock になる**：API キーが未設定、またはエラーが続くと自動的に mock で続行します。端末の警告（HTTP 401 など）を確認してください

## API キーの扱い

API キーは環境変数で渡してください。コードやリポジトリには含めません。`.gitignore` で `.env` と `*.key`、ログを除外しています。

## ライセンス

Apache-2.0。以下の Apache-2.0 のコードをもとにしています。

- `tidy_robot/kinematics.py`：『ROS 2とPythonで作って学ぶAIロボット入門 改訂第2版』chapter6 `crane_plus_commander/kinematics.py`（AI-Robot-Book-Humble）
- `launch/tidy_sim.launch.py`：`crane_plus_gazebo/no_moveit_crane_plus_with_table.launch.py`（RT Corporation）

関連研究：TidyBot（Wu et al., 2023, [arXiv:2305.05658](https://arxiv.org/abs/2305.05658)）— LLM で片付けの好みを推定する研究。本作品はその考え方をシミュレーション上で、整理（要・不要）を含めて小さく実装したものです。
