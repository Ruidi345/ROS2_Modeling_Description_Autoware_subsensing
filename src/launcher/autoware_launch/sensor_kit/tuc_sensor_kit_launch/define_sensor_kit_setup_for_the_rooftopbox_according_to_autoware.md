
# Define Sensor Kit Setup for the Rooftop Box According to Autoware

## Table of Contents

1. [TUC Sensor Kit Structure](#1-tuc-sensor-kit-structure)
   - [Filepath Structure](#filepath-structure)
2. [TUC Sensor Kit Building](#2-tuc-sensor-kit-building)
   - [Repository Cloning](#1-repository-cloning)
   - [Run the Docker Container](#2-run-the-docker-container-repository-of-autoware)
   - [Build the TUC Sensor Kit](#3-build-the-tuc-sensor-kit)
   - [Provide a Default Calibration File](#4-provide-a-default-calibration-file)
   - [Obtain Missing Configuration Files](#5-obtain-the-missing-configuration-files)
   - [Build Individual Parameters](#6-build-the-individual-parameters)
3. [TUC Sensor Kit Run](#3-tuc-sensor-kit-run)
4. [Useful References](#4-useful-references)

## 1) TUC Sensor Kit Structure

The TUC Sensor Kit consists of the following components:

- `tuc_common_sensor_launch`
- `tuc_sensor_kit_description`
- `tuc_sensor_kit_launch`

### Filepath Structure

```
<Autoware_Workspace_Dir>/
  └─ src/
     └── sensor_kit/
          └── tuc_sensor_kit/
          └── tuc_sensor_kit_launch/
              ├── tuc_common_sensor_launch/
              │   ├── CMakeLists.txt
              │   ├── launch/
              │   │   └── carla_lidar_node_container.launch.py
              │   └── package.xml
              ├── tuc_sensor_kit_description/
              │   ├── CMakeLists.txt
              │   ├── config/
              │   │   ├── sensor_kit_calibration.yaml
              │   │   └── sensors_calibration.yaml
              │   ├── package.xml
              │   └── urdf/
              │       ├── sensor_kit.xacro
              │       └── sensors.xacro
              └── tuc_sensor_kit_launch/
                  ├── CMakeLists.txt
                  ├── config/
                  │   ├── diagnostic_aggregator/
                  │   │   └── sensor_kit.param.yaml
                  │   └── dummy_diag_publisher/
                  │       └── sensor_kit.param.yaml
                  ├── data/
                  │   └── traffic_light_camera.yaml
                  ├── launch/
                  │   ├── gnss.launch.xml
                  │   ├── imu.launch.xml
                  │   ├── lidar.launch.xml
                  │   ├── pointcloud_preprocessor.launch.py
                  │   └── sensing.launch.xml
                  └── package.xml
```

## 2) TUC Sensor Kit Building

### 1. Repository Cloning

Clone the TUC Sensor Kit repository from GitLab. Authentication credentials are required. Use your `<TUC username>` and `<password>`:

```bash
$ cd ~/<Autoware Workspace>/autoware/src
$ git clone https://gitlab.tu-clausthal.de/isse/rg-dacs/tuc_sensor_kit_launch.git
```

### 2. Run the Docker Container Repository of Autoware

Start the Autoware Docker container and source the Autoware workspace:

```bash
$ rocker --network=host -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --nvidia --volume /home/<...>/<Path_to_Autoware_Workspace_Dir> -- ghcr.io/autowarefoundation/autoware:humble-2024.01-cuda-amd64

$ cd /home/<.../Path_to_Autoware_Workspace_Dir>/autoware

$ source install/setup.bash
```

### 3. Build the TUC Sensor Kit

Compile the TUC Sensor Kit within the Autoware workspace:

```bash
$ cd /home/<.../Path_to_Autoware_Workspace_Dir>/
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to tuc_sensor_kit_description tuc_sensor_kit_launch
```

### 4. Provide a Default Calibration File

Create a directory for the default individual parameter configurations:

```bash
$ mkdir -p /home/<.../Path_to_Autoware_Workspace_Dir>/autoware/install/individual_params/share/individual_params/config/default/tuc_sensor_kit
```

### 5. Obtain Missing Configuration Files

Copy the required calibration files into the newly created directory:

```bash
$ cp /home/<...>/<Path_to_Autoware_Workspace_Dir>/autoware/src/sensor_kit/tuc_sensor_kit/tuc_sensor_kit_launch/tuc_sensor_kit_description/config/sensors_calibration.yaml /home/<.../Path_to_Autoware_Workspace_Dir>/autoware/install/individual_params/share/individual_params/config/default/tuc_sensor_kit/

$ cp /home/<...>/<Path_to_Autoware_Workspace_Dir>/autoware/src/sensor_kit/tuc_sensor_kit/tuc_sensor_kit_launch/tuc_sensor_kit_description/config/sensor_kit_calibration.yaml /home/<.../Path_to_Autoware_Workspace_Dir>/autoware/install/individual_params/share/individual_params/config/default/tuc_sensor_kit/
```

### 6. Build the Individual Parameters

Compile the `individual_params` package:

```bash
$ colcon build --packages-select individual_params
```

## 3) TUC Sensor Kit Run

To use the TUC Sensor Kit, specify it as the `sensor_model` argument, the vehicle as `vehicle_model`, and provide the path to the map as `map_path`:

```bash
$ ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/home/<.../Path_to_Autoware_Workspace_Dir>/Maps/Town10 vehicle_model:=tuc_vehicle sensor_model:=tuc_sensor_kit
```

## 4) Useful References

- [Autoware-CARLA Bridge Setup Documentation](https://gitlab.tu-clausthal.de/isse/rg-dacs/autoware-carla-operation/-/blob/docs/docs/autoware_bridge_carla_setup.md?ref_type=heads)
- [Autoware Sensor Model Creation Guide](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-and-sensor-model/creating-sensor-model/)
