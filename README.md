# ROS 2 Door Monitor

English · [日本語](docs/README.ja.md)

A ROS 2 node that monitors a known doorway's open/closed or blocked state using
its map position and LaserScan data. This private standalone implementation
retains the purpose of `pylot_door_monitor` while replacing localization-specific
coupling with TF. It requires neither EMCL nor PyLoT's Robot facade.

## Try without ROS

Requires Python 3.10+. Run these commands from the repository root.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
python -m pytest -q
python -m door_monitor.demo
```

The demo prints `TRANSITIONING → CLOSED → TRANSITIONING → OPEN → UNKNOWN`.
`door_monitor.core` uses only the standard library and accepts explicit timestamps
for deterministic tests. When ROS is unavailable, only the ROS integration test
is skipped.

## ROS 2 Jazzy

Place the repository in a regular workspace's `src` directory, install its
dependencies and build it:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select door_monitor
source install/setup.bash
ros2 launch door_monitor door_monitor.launch.py params_file:=/absolute/path/to/door.yaml
```

Inputs are `sensor_msgs/LaserScan` and the TF needed to express the known doorway
center from `reference_frame` in the scan's `header.frame_id`. The transform is
looked up at the scan timestamp, accounting for robot yaw. `door_x` and `door_y`
are in metres. The configured point has z=0 in the reference frame, and planar
scanning is assumed. Parameters are read at startup; restart the node after
changing them.

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

Outputs are `/door_monitor/state` (`std_msgs/String`), `/door_monitor/is_open`
(`Bool`), `/door_monitor/debug` (`String` JSON) and `/door_monitor/marker`
(an RViz Marker). States are `UNKNOWN`, `TRANSITIONING`, `OPEN` and `CLOSED`.
`is_open=true` is emitted only for a stable OPEN state. Use
`/door_monitor/set_enabled` (`std_srvs/SetBool`) to enable or disable monitoring;
either change resets the state to UNKNOWN.

The doorway is OPEN when the nearest valid return in its angular cone lies
beyond the distance to its center plus `clearance_m`; otherwise it is CLOSED.
People and other obstacles also produce CLOSED. This does not recognize door
geometry or guarantee safe passage. Partial cone coverage, too few valid returns,
excessive proximity, missing TF or stale timestamps produce UNKNOWN. A scan
containing only +inf returns is not evidence of OPEN. Large spaces without a
measurable background may therefore remain UNKNOWN.

A monotonic wall-clock watchdog resets the state to UNKNOWN after input stops.
Its timer continues running when the ROS simulation clock is paused. Stable
states require continuous observations; gaps and clock resets cannot accumulate
dwell time. The original package's localization pose republishing and initialpose
control are outside this node's scope. Remap topics if you use legacy names such
as `/door_monitor/door_state`.

## Validation

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest -q
```

Alongside unit tests, an isolated DDS domain supplies static TF and synthetic
scans to a real node in another process, verifying CLOSED → OPEN → UNKNOWN.
No robot or physical LiDAR is needed. Performance on real doors and sensors
has not been measured.

## License and provenance

Apache-2.0. See [PROVENANCE.md](PROVENANCE.md) for the original implementation
and extraction changes.
