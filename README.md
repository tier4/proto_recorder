# proto_recorder

A ROS 2 component node for recording messages to rosbag with topic rate monitoring, automatic pause/resume control, and diagnostics publishing.

## Requirements

- ROS 2 (Humble or later)
- rosbag2
- yaml-cpp

## Installation

```bash
# Clone into your colcon workspace
cd ~/colcon_ws/src
git clone https://github.com/tier4/proto_recorder.git

# Install dependencies
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -y

# Build
colcon build --packages-up-to proto_recorder
```

## Usage

### Launch

```bash
# Source the workspace
source install/setup.bash

# Launch with default parameters
ros2 launch proto_recorder proto_recorder.launch.xml

# Launch with custom parameters
ros2 launch proto_recorder proto_recorder.launch.xml \
  uri:=my_recording \
  storage_id:=mcap \
  start_recording:=true
```

### Run as a component node

```bash
ros2 run proto_recorder proto_recorder_node \
  --ros-args -p topics_file:=/path/to/record_topics.yaml
```

## Parameters

### Storage

| Parameter                | Type   | Default           | Description                                                          |
| ------------------------ | ------ | ----------------- | -------------------------------------------------------------------- |
| `storage_id`             | string | `mcap`            | Storage plugin (`sqlite3`, `mcap`, etc.)                             |
| `uri`                    | string | `proto_recording` | Output directory prefix for recorded bag (timestamped automatically) |
| `max_bagfile_size`       | int    | `0`               | Maximum bag file size in bytes before splitting (`0` = no limit)     |
| `max_bagfile_duration`   | int    | `0`               | Maximum duration in seconds before splitting (`0` = no limit)        |
| `max_cache_size`         | int    | `0`               | Maximum cache size in bytes (`0` = no caching)                       |
| `storage_preset_profile` | string | `""`              | Storage preset (`fastwrite`, `zstd_fast`, `zstd_small` for MCAP)     |
| `storage_config_uri`     | string | `""`              | Path to storage configuration YAML file                              |

### Compression

| Parameter                | Type   | Default | Description                                               |
| ------------------------ | ------ | ------- | --------------------------------------------------------- |
| `compression_mode`       | string | `""`    | Compression mode (`none`, `file`, `message`)              |
| `compression_format`     | string | `""`    | Compression format (e.g., `zstd`)                         |
| `compression_queue_size` | int    | `1`     | Number of files/messages queued for compression           |
| `compression_threads`    | int    | `0`     | Number of compression threads (`0` = number of CPU cores) |

### Recording

| Parameter               | Type   | Default | Description                                            |
| ----------------------- | ------ | ------- | ------------------------------------------------------ |
| `topics_file`           | string | `""`    | Path to YAML file containing topics to record          |
| `serialization_format`  | string | `cdr`   | Serialization format                                   |
| `start_paused`          | bool   | `false` | Start recording in paused state                        |
| `start_recording`       | bool   | `true`  | Start recording immediately when node starts           |
| `wait_for_stable_rates` | bool   | `false` | Wait for all topic rates to stabilize before recording |

### Diagnostics

| Parameter                | Type   | Default          | Description                                  |
| ------------------------ | ------ | ---------------- | -------------------------------------------- |
| `rate_check_window_size` | int    | `10`             | Number of messages used for rate calculation |
| `diagnostics_period`     | double | `1.0`            | Period in seconds for publishing diagnostics |
| `hardware_id`            | string | `proto_recorder` | Hardware ID for diagnostics                  |

## Topics file format

Define topics to record in a YAML file with optional rate monitoring thresholds:

```yaml
topics:
  - name: /topic1
    min_rate: 18.0
    max_rate: 21.0
  - name: /topic2
    min_rate: 18.0
    max_rate: 21.0
```

## Subscribed topics

| Topic           | Type                | Description                                  |
| --------------- | ------------------- | -------------------------------------------- |
| `~/input/start` | `std_msgs/msg/Bool` | Start (`true`) or stop (`false`) recording   |
| `~/input/pause` | `std_msgs/msg/Bool` | Pause (`true`) or resume (`false`) recording |

## Published topics

| Topic             | Type                                     | Description                                          |
| ----------------- | ---------------------------------------- | ---------------------------------------------------- |
| `~/output/status` | `proto_recorder_msgs/msg/RecorderStatus` | Periodic recording status and topic rate diagnostics |

## Default topic remappings (in launch file)

| Node topic        | Remapped to        |
| ----------------- | ------------------ |
| `~/input/start`   | `/recorder/start`  |
| `~/input/pause`   | `/recorder/pause`  |
| `~/output/status` | `/recorder/status` |

## QoS adaptation

The node automatically adapts subscription QoS to match publishers:

- If all publishers use RELIABLE, the subscription uses RELIABLE
- If any publisher uses BEST_EFFORT, the subscription uses BEST_EFFORT
- Same logic applies to durability (TRANSIENT_LOCAL vs VOLATILE)
