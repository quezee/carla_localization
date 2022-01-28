# Scan Matching Localization

## Goal

Localize a car driving in simulation for at least 170m from the starting position and never exceeding a distance pose error of 1.2m. The simulation car is equipped with a lidar, provided by the simulator at regular intervals are lidar scans. There is also a point cloud map [map.pcd](https://github.com/quezee/carla_localization/blob/main/map.pcd) already extracted from the CARLA simulator.

## Developed solution

Normal Distribution Transform + Kalman Filter.

## Result

Pose error doesn't exceed 1.2m for at least 170m ride with speed level up to 4.

![](https://github.com/quezee/carla_localization/blob/main/result.gif)

## Usage

**Warning**: project parameters are finetuned to get accurate localization inside of Udacity virtual environment. Since accuracy depends on host hardware as well, you may not get similar results.

1. Pull CARLA simulator image (server)
```
docker pull carlasim/carla:0.9.13
```
2. Launch CARLA server container
```
docker run --privileged --gpus all --net=host carlasim/carla:0.9.13 /bin/bash ./CarlaUE4.sh -RenderOffScreen
```
3. Build project image (client) from repo root. It will test connection with CARLA container by the end of build process.
```
docker build -t carla_localization --network=host -f docker/Dockerfile .
```
4. Re-run server container and launch client container
```
docker run --gpus all --net=host -e DISPLAY=$DISPLAY carla_localization
```

Tap the UP key up to 4 times to gain some speed and use LEFT/RIGHT to steer the wheel.
