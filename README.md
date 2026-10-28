# duatic_gazebo
[![Jazzy](https://github.com/Duatic/duatic_gazebo/raw/badges/jazzy.svg)](https://github.com/Duatic/duatic_gazebo/actions/workflows/ci.yml)
[![Kilted](https://github.com/Duatic/duatic_gazebo/raw/badges/kilted.svg)](https://github.com/Duatic/duatic_gazebo/actions/workflows/ci.yml)
[![Lyrical](https://github.com/Duatic/duatic_gazebo/raw/badges/lyrical.svg)](https://github.com/Duatic/duatic_gazebo/actions/workflows/ci.yml)
[![Rolling](https://github.com/Duatic/duatic_gazebo/raw/badges/rolling.svg)](https://github.com/Duatic/duatic_gazebo/actions/workflows/ci.yml)

## Overview

Provides Gazebo simulation worlds, robot spawning, and ROS-Gazebo bridge configuration for Duatic robots. Includes empty world and warehouse environment with AWS RoboMaker assets.

# License

The contents are licensed under the BSD-3-Clause  [license](LICENSE).\
Images in this repository are to be licensed separately if you want to use them for any other usecase than forking this repository. Please open an issue in order to get in touch with us.


## Usage

### Launch Simulation Environment

#### Using ROS Node (Recommended)

The `start_sim` node provides a managed way to start and stop the simulation. It automatically ensures the simulation container is properly shut down when the node terminates.

```bash
# Empty world (default)
ros2 run duatic_gazebo start_sim.py

# Warehouse world
ros2 run duatic_gazebo start_sim.py --ros-args -p world:=warehouse

# Headless simulation
ros2 run duatic_gazebo start_sim.py --ros-args -p world:=warehouse -p headless:=true
```

**Benefits:**
- Automatic cleanup: Docker container is stopped when node shuts down (Ctrl+C)
- ROS integration: Works as a standard ROS node with parameters
- Process management: Proper lifecycle handling

#### Using Launch File Directly

```bash
# Empty world (default)
ros2 launch duatic_gazebo gazebo.launch.py

# Warehouse world
ros2 launch duatic_gazebo gazebo.launch.py world:=warehouse

# Headless simulation
ros2 launch duatic_gazebo gazebo.launch.py world:=warehouse headless:=true
```

### Spawn Robot

Prerequisite: URDF is being published with the topic /{namespace}/robot_description

```bash
# Basic spawn
ros2 launch duatic_gazebo spawn.launch.py namespace:=robot1 x:=0 y:=0 z:=1 yaw:=0

# Multiple robots
ros2 launch duatic_gazebo spawn.launch.py namespace:=alpha x:=0 y:=0 z:=1 yaw:=0 &
ros2 launch duatic_gazebo spawn.launch.py namespace:=beta x:=2 y:=2 z:=1 yaw:=1.57 &
```

### ROS-Gazebo Bridge

```bash
# Launch bridge with config file
ros2 launch duatic_gazebo ros_gz_bridge.launch.py \
  namespace:=robot1 \
  world:=warehouse \
  config_file:=/path/to/bridge_config.yaml
```

## Launch Files

| File | Description |
|------|-------------|
| `gazebo.launch.py` | Main simulation environment |
| `spawn.launch.py` | Robot spawning interface |
| `ros_gz_bridge.launch.py` | ROS-Gazebo topic bridge |

## Nodes

### start_sim.py

ROS 2 node that manages the simulation Docker container lifecycle.

**Parameters:**
- `world` (string, default: "empty"): World name to load
- `headless` (bool, default: false): Run simulation without GUI

**Features:**
- Starts simulation container using docker compose
- Automatically stops container on node shutdown
- Proper cleanup with Ctrl+C
- ROS parameter support

**Example:**
```bash
ros2 run duatic_gazebo start_sim.py --ros-args -p world:=warehouse -p headless:=false
```

## Parameters

### gazebo.launch.py
| Parameter | Default | Description |
|-----------|---------|-------------|
| `world` | `"empty"` | World name (empty, warehouse) |
| `headless` | `false` | Run without GUI |
| `gz_models_path` | `""` | A ':'-separated list of Gazebo resource search paths |

### spawn.launch.py
| Parameter | Default | Description |
|-----------|---------|-------------|
| `namespace` | `"empty_namespace"` | Robot namespace |
| `x`, `y`, `z` | `0.0` | Spawn position |
| `yaw` | `0.0` | Spawn orientation |

### ros_gz_bridge.launch.py
| Parameter | Default | Description |
|-----------|---------|-------------|
| `namespace` | `"empty_namespace"` | Robot namespace |
| `world` | `"undefined"` | World name |
| `config_file` | - | Bridge configuration file |

## Available Worlds

- **duatic_empty**: Minimal environment for basic testing
- **warehouse**: AWS RoboMaker warehouse with shelves, pallets, and clutter
- **bag_lifting**: Demo booth with a wall, pallet, table, and rice bags, used for pick/place demos
- **bag_sorting**: Six bags on three tables, can be sorted onto the pallets
- **bag_sorting_single**: One bag on the middle table, for general application testing
- **klt_depalletizing**: Closed 10 x 8 m room with 6 KLT boxes in a single layer on the floor around (3, 0), 2 cm apart, and four stations at y = +/-3 with stacked boxes for the known object skills (blocked handle, box on a handle, offset stack, single box)

A world per scenario beats clearing objects at runtime: with the objects already in place,
a fresh simulation is the reset. Occupancy maps live in `maps/`, one pair per world under
the same basename, and are derived from the world geometry, so move a wall in the `.sdf`
and the map is silently wrong. Maps recorded at a real site are instance data and live in
`/srv/duatic/maps`.

## Assets

### AWS RoboMaker Models
- Warehouse infrastructure (shelves, walls, ground)
- Props (buckets, lamps, trash cans, pallet jacks)
- Clutter objects for realistic environments

### Bag Lifting Demo Models
- Demo booth (`booth`)
- Pallet, table, and rice bag props (`pallet`, `table`, `bag`)


# Contributing

Please see the [Contributing guide](./CONTRIBUTING.md)
