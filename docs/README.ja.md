# ROS 2 Door Monitor

[English](../README.md) · 日本語

地図上の既知のドア位置とLaserScanから、開閉・通路の遮蔽状態を監視するROS 2ノードです。
`pylot_door_monitor` の目的を引き継ぎ、自己位置推定への依存をTFへ置き換えた
privateの独立実装です。EMCLやPyLoTのRobotクラスは不要です。

## ROSなしで試す

Python 3.10+。リポジトリのルートで実行します。

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
python -m pytest -q
python -m door_monitor.demo
```

デモは `TRANSITIONING → CLOSED → TRANSITIONING → OPEN → UNKNOWN` を出します。
`door_monitor.core` は標準ライブラリのみで、実時刻を渡して決定的にテストできます。
ROSが無い環境ではROS結合テストだけをスキップします。

## ROS 2 Jazzy

通常のワークスペースのsrcにリポジトリを置き、依存を準備してビルドします。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select door_monitor
source install/setup.bash
ros2 launch door_monitor door_monitor.launch.py params_file:=/absolute/path/to/door.yaml
```

必要な入力は `sensor_msgs/LaserScan` と、`reference_frame` からスキャンの
`header.frame_id` へのTFです。TFはスキャン時刻で取得します。地図のドア中心
`door_x`, `door_y`（メートル）をスキャン座標へ変換するため、自己位置のyawを
無視することはありません。基準フレーム内のドア中心はz=0、スキャンは平面を想定します。
パラメータは起動時設定です。変更する場合はノードを再起動してください。

| Parameter | Default | Meaning |
|---|---|---|
| `reference_frame` | `map` | Door coordinates frame |
| `scan_topic` | `/scan` | LaserScan input |
| `door_x`, `door_y` | 2.0, 0.0 | Known door center, metres |
| `width_m` | 0.8 | Doorway cone width |
| `clearance_m` | 0.2 | Required finite return distance beyond door center |
| `stable_sec` | 0.5 | Continuous evidence needed for a stable state |
| `stale_sec` | 0.5 | Wall-time input watchdog |
| `max_scan_age_sec` | 0.5 | Maximum ROS timestamp age |
| `max_future_sec` | 0.1 | Maximum permitted future timestamp |
| `min_valid_samples` | 3 | Minimum finite returns in cone |
| `min_valid_fraction` | 0.6 | Minimum fraction of valid cone returns |

出力は `/door_monitor/state` (`std_msgs/String`)、`/door_monitor/is_open` (`Bool`)、
`/door_monitor/debug` (`String` JSON)、`/door_monitor/marker`（RViz用Marker）。
`state` は `UNKNOWN`, `TRANSITIONING`, `OPEN`, `CLOSED` のいずれかです。
`is_open=true` は安定したOPENだけです。サービス `/door_monitor/set_enabled`
(`std_srvs/SetBool`) で有効・無効を切り替え、切替時はUNKNOWNに戻します。

ドア方向の扇形内で最も近い有効レンジが「ドア中心までの距離＋clearance」より
遠いとOPEN、近いとCLOSEDです。人や物が遮ってもCLOSEDになります。
ドアの形状認識や通過の安全保証は行いません。扇形の両端が見えない、無効レンジが多い、
距離が近すぎる、TFが無い、時刻が古い場合はUNKNOWNです。+infだけのスキャンを
OPENとは扱いません。壁まで測れない広い空間ではUNKNOWNになり得ます。

入力が途切れるとmonotonic wall clockのwatchdogでUNKNOWNへ戻ります。
watchdogのタイマーはROSのシミュレーション時計が停止しても動きます。
連続した観測でのみ確定し、入力断や時計リセットをまたいで安定時間を稼ぎません。
元パッケージの自己位置再配信・initialpose制御はこのノードの責務に含めていません。
既存の `/door_monitor/door_state` などを使う場合はremapしてください。

## 検証

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest -q
```

単体テストに加え、独立したDDSドメインで静的TFと合成スキャンを配信し、
実際の別プロセスのノードがCLOSED→OPEN→UNKNOWNへ遷移することを確認します。
ロボットや実LiDARは不要です。実ドア・実センサでの性能測定は未実施です。

## ライセンスと出典

Apache-2.0。元実装と今回の構成変更は [PROVENANCE.md](../PROVENANCE.md) を参照。
