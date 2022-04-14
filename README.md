# CARLA Localization

## Goal

Build localization solution for a car driving in CARLA simulator with respect to pre-extracted [point cloud map](https://github.com/quezee/carla_localization/blob/main/data/map1.pcd).

## Developed solution

Unscented Kalman Filter fusing IMU, GNSS and lidar sensors.<br>
Lidar scans are matched with Normal Distribution Transform.

Project parameters are configured through [config.cfg](https://github.com/quezee/carla_localization/blob/main/config.cfg)

## Demo result

<img src="https://github.com/quezee/carla_localization/blob/main/demo1.gif" width="600">

## Usage

1. Pull CARLA simulator image (server)
```
docker pull carlasim/carla:0.9.13
```
2. Launch CARLA server container
```
docker run --privileged --gpus all --net=host carlasim/carla:0.9.13 /bin/bash ./CarlaUE4.sh -RenderOffScreen
```
3. Build project image (client) from repo root. It will test connection with CARLA server by the end of build process.
```
docker build -t carla_localization --network=host -f docker/Dockerfile .
```
4. Re-run server container and launch client container
```
docker run --gpus all --net=host -e DISPLAY=$DISPLAY carla_localization
```
Before 4. you may need to run ```xhost +si:localuser:root``` to let docker access your display.
