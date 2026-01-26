# Docker Usage Guide

This guide explains how to build the Docker image and run the different available modes using the provided `Dockerfile` and `entrypoint.sh`.

## 1. Building the Docker Image

To build the Docker image, run the following command in the root directory of the project:

```bash
docker build -t kmc_unita .
```

You can replace `kmc_unita` with any name you prefer for your image.

## 2. Running the Container

The container's behavior is controlled by environment variables passed to the `docker run` command using the `-e` flag.

### Available Environment Variables

| Variable | Default | Description |
| :--- | :--- | :--- |
| `RUN_MODE` | `algorithm` | Determines the operation mode: `algorithm` (or `team`), `sim` (or `simulator`). |
| `PROBLEM_ID` | `3` | **(Algorithm Mode)** Selects the specific task to run: `3` (Competition), `viz` (Map Visualizer). |
| `TARGET_CAV` | `all` | **(Algorithm Mode)** Selects which vehicle to control: `all`, `cav1`, `cav2`, `cav3`, `cav4`. |
| `SINGLE_CAV` | `false` | **(Algorithm Mode)** Set to `true` if targeting a single vehicle context. |
| `ROS_DOMAIN_ID` | `100` | Sets the ROS 2 Domain ID for communication. |

---

### A. Run the Simulator (`RUN_MODE=sim`)

To start the simulation environment:

```bash
docker run -it --rm \
    --net=host \
    -e RUN_MODE=sim \
    kmc_unita
```

*   `--net=host`: Required for ROS 2 communication between containers (simulator <-> algorithm).

### B. Run the Algorithm (`RUN_MODE=algorithm`)

This is the default mode. It runs the team's pure pursuit controller or other logic.

#### 1. Run Competition Task (Problem 3)

To run the main competition logic:

```bash
docker run -it --rm \
    --net=host \
    -e RUN_MODE=algorithm \
    -e PROBLEM_ID=3 \
    -e CAV1_ID=5 \
    -e CAV2_ID=6 \
    -e CAV3_ID=7 \
    -e CAV4_ID=8 \
    -e TARGET_CAV=all \
    kmc_unita
```

**Optional: Controlling Specific Vehicles**

To run the algorithm for a specific vehicle (e.g., `cav1`):

```bash
docker run -it --rm \
    --net=host \
    -e RUN_MODE=algorithm \
    -e CAV1_ID=5 \
    -e CAV2_ID=6 \
    -e CAV3_ID=7 \
    -e CAV4_ID=8 \
    -e SINGLE_CAV=true \
    -e TARGET_CAV=cav1 \
    kmc_unita
```

#### 2. Run Map Visualizer (`PROBLEM_ID=viz`)

To visualize the map using RViz (requires a GUI environment or X11 forwarding):

```bash
xhost +local:root # Allow docker to access X server (if running locally on Linux)
docker run -it --rm \
    --net=host \
    -e RUN_MODE=algorithm \
    -e PROBLEM_ID=viz \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    kmc_unita
```

## Summary of Commands

| Goal | Command |
| :--- | :--- |
| **Build Image** | `docker build -t kmc_unita .` |
| **Run Simulator** | `docker run -it --rm --net=host -e RUN_MODE=sim kmc_unita` |
| **Run Algorithm** | `docker run -it --rm --net=host -e RUN_MODE=algorithm kmc_unita` |
| **Run Visualizer** | `docker run -it --rm --net=host -e RUN_MODE=algorithm -e PROBLEM_ID=viz -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix kmc_unita` |
