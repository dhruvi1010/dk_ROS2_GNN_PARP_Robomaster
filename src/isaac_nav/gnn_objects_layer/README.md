# gnn_objects_layer

A custom Nav2 costmap layer plugin for ROS 2 that converts tracked semantic object polygons into navigation costs for real-time obstacle avoidance.

## Overview

`gnn_objects_layer` bridges learned perception and classical navigation.

The layer subscribes to tracked polygon messages produced by the perception pipeline, transforms them into the active costmap frame, removes stale objects based on configurable decay times, rasterizes the polygons into the costmap, and applies class-aware inflation/cost behavior. The resulting costs are merged into the Nav2 master costmap so standard planners can react to semantic obstacles such as robots, workstations, forklifts, and boundaries.

## Main Functionality

- Subscribes to tracked semantic polygons from a ROS 2 topic
- Transforms incoming polygons into the global costmap frame using TF2
- Maintains a time-aware internal buffer of active polygons
- Removes stale polygons using configurable decay times
- Rasterizes obstacle polygons into the costmap grid
- Applies semantic class-dependent inflation behavior
- Updates the Nav2 master costmap with dynamic obstacle costs
- Logs timing and obstacle statistics for debugging and evaluation

## Use Case

This package is designed for scenarios where a perception stack already produces tracked semantic obstacle polygons and the navigation stack must use them directly for obstacle avoidance.

Typical pipeline:

```text
Perception / Tracking Node
        |
        v
 /tracked_polygons
        |
        v
 GNNObjectsLayer
        |
        v
 Nav2 Costmap
        |
        v
 Planner / Controller
```

## Supported Semantic Labels

The layer currently uses semantic labels such as:

- `WORKSTATION`
- `ROBOT`
- `BOUNDARY`
- `FORKLIFT`

These labels can be configured with different:

- decay times
- inflation radii
- cost behaviors

## Package Structure

```text
gnn_objects_layer/
├── include/gnn_objects_layer/
│   └── gnn_objects_layer.hpp
├── src/
│   └── gnn_objects_layer.cpp
├── launch/
├── scripts/
├── CMakeLists.txt
├── package.xml
└── plugin.xml
```

## Core Class

### `GNNObjectsLayer`

This package implements a custom Nav2 costmap layer by extending:

```cpp
nav2_costmap_2d::CostmapLayer
```

Key responsibilities of the class:

- plugin initialization
- polygon message subscription
- TF transformation into costmap frame
- stale obstacle cleanup
- costmap bounds update
- polygon rasterization and inflation
- writing costs into the master grid

## Input Topic

### Subscribed topic

By default, the layer subscribes to:

```text
/tracked_polygons
```

### Expected message type

```text
gnn_interfaces/msg/TrackedPolygon
```

The incoming message is expected to contain:

- message header with timestamp and frame
- polygon points
- semantic label
- confidence score

## Costmap Behavior

For each incoming tracked polygon, the layer:

1. checks message age
2. transforms polygon points into the global costmap frame
3. stores the polygon in an internal buffer
4. removes expired polygons during update
5. converts world coordinates to map cells
6. fills the polygon region in the costmap
7. applies optional class-specific inflation
8. writes the resulting costs into the Nav2 master grid

This allows semantic perception results to influence path planning without modifying the planner itself.

## Parameters

The exact parameter file may vary by deployment, but the layer supports configuration for:

### General

- `enabled`
- `topic`
- `decay_time`

### Semantic behavior

- `label_decay_times`
- `label_inflation_radii`

### Notes

- `decay_time` acts as the fallback expiration time for polygons
- `label_decay_times` overrides decay per object class
- `label_inflation_radii` controls semantic inflation per object class

## Example Configuration

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["static_layer", "inflation_layer", "gnn_objects_layer"]

      gnn_objects_layer:
        plugin: "gnn_objects_layer::GNNObjectsLayer"
        enabled: true
        topic: "/tracked_polygons"
        decay_time: 5.0
        label_decay_times: [0.0, 5.0, 1.0, 10.0, 2.0]
        label_inflation_radii: [0.0, 0.4, 0.3, 0.5, 0.8]
```

Adjust label ordering to match your message encoding.

## Build

Inside your ROS 2 workspace:

```bash
colcon build --packages-select gnn_objects_layer
source install/setup.bash
```

## Plugin Export

The plugin is exported through `pluginlib` and can be loaded by Nav2 as a custom costmap layer.

## Logging

The package includes CSV-based logging for:

- costmap update timing
- polygon age / delay
- label counts
- first-use latency metrics

These logs can help analyze perception-to-navigation latency and obstacle update behavior.

## Strengths of This Layer

- clean integration into Nav2 through the costmap layer interface
- direct bridge from semantic perception to navigation
- support for dynamic obstacle decay
- class-aware inflation and obstacle handling
- no planner modification required
- useful for shared perception and semantic navigation experiments

## Limitations

Current implementation assumptions include:

- tracked polygons are already produced by an upstream perception/tracking module
- TF transforms between polygon frame and costmap frame must be available
- semantic labels must match the expected internal label mapping
- logging paths and deployment settings may need adaptation for portability

## Future Improvements

Possible extensions include:

- cleaner parameter namespacing
- configurable logging path
- stronger class-specific cost assignment
- improved lifecycle/error handling
- better separation of logging and core layer logic
- visualization helpers for debugging semantic obstacle overlays

## Summary

`gnn_objects_layer` is a custom ROS 2 Nav2 costmap plugin that converts tracked semantic polygons into dynamic navigation costs, enabling planners to react to learned perception outputs in real time.
