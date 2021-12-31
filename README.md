# Scan Matching Localization

## Goal

Localize a car driving in simulation for at least 170m from the starting position and never exceeding a distance pose error of 1.2m. The simulation car is equipped with a lidar, provided by the simulator at regular intervals are lidar scans. There is also a point cloud map [map.pcd](https://github.com/quezee/carla_localization/blob/main/map.pcd) already extracted from the CARLA simulator.

## Developed solution

Normal Distribution Transform + Kalman Filter.

## Result

Pose error doesn't exceed 1.2m for at least 170m ride with speed level up to 4.
(https://github.com/quezee/carla_localization/blob/main/result.gif)

## Usage

Launch CARLA simulator server:
```
./run_carla.sh
```

Compile and the project:

```
cmake .
make
./cloud_loc
```

Tap the UP key up to 4 times to gain some speed and use LEFT/RIGHT to steer the wheel.
**Warning**: project parameters are finetuned to get accurate localization inside of Udacity virtual environment. Since accuracy depends on host hardware as well, you may not get similar results.