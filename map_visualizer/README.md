# Map Visualizer

This package visualizes the map used in `Mobility_Challenge_Simulator/simulator`.

## Description

It reads the `path.json` file from the `simulator` package's resource directory and publishes it as `visualization_msgs/MarkerArray` on the topic `/map_markers`.

## CAV Visualization

The `cav_visualizer_node` subscribes to `PoseStamped` topics (default: `/CAV_01` to `/CAV_04`) and visualizes the vehicles as Cube markers in RViz.
It loads vehicle dimensions from `Mobility_Challenge_Simulator/src/simulator/resource/config.ini` and handles the non-standard orientation format (Euler angles in Quaternion field).

## Usage

1. Build the package:
   ```bash
   colcon build --packages-select map_visualizer
   source install/setup.bash
   ```

2. Run the launch file (starts map and 4 CAV visualizers):
   ```bash
   ros2 launch map_visualizer visualize.launch.py
   ```

3. In RViz:
   - Set **Fixed Frame** to `map`.
   - Add a **MarkerArray** display for `/map_markers`.
   - Add a **Marker** display for `/cav_markers` (Namespace: `cavs`).
