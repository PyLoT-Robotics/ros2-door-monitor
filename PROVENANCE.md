# Source provenance

Extracted from the private PyLoT-Robotics/pylot_robocup2025 repository.

- Source commit: `55960a6c7bcdaff870a593dedbbb46a7bbeee375`
- Source path: `localization/pylot_door_monitor`
- Source URL: https://github.com/PyLoT-Robotics/pylot_robocup2025/tree/55960a6c7bcdaff870a593dedbbb46a7bbeee375/localization/pylot_door_monitor

Original authorship and inline attribution are retained. This private extraction adds
independent packaging, examples, tests and documentation. External libraries,
models, corpora and recordings are not bundled and retain their own licenses.

The core and ROS adapter were reimplemented during extraction. The legacy pose
republisher and initialpose control were omitted; TF-at-scan-time geometry,
continuous dwell, validity checks and monotonic stale-input handling were added.
