# hyundai_amr

This repository contains code for a Hyudai hyundai_amr SW  with ROS2 system (humble). 

- PC
  - TBD...

## Requirements
* Docker env
* Nvidia docker env
* Nvidia driver
* CUDA
### CUDA setup
*  For the cuda build, there's a config related with CUDA in dockerfile
*  ENV PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
*  The CUDA must be installed in the local env 
*  check whether NVIDIA Driver is working or not ($ nvidia-smi)
*  check whether CUDA is working or not ($ nvcc -V)
*  check ($/usr/local/cuda) available


## Run in the Docker
### Installation
```
$ mkdir -p ~/amr_ws/src
$ cd /amr_ws/src
```

```
$ git clone https://github.com/sungwon-Nah/hyundai_amr.git
```

```
$ cd hyundai_amr/operations/scripts/
```

```
$ ./hyundai_docker_build.sh
```
** It may take some time....


### Get started
```
$ ./hyundai_docker_up.sh
```

### When you first run the container 
```
$ source /opt/ros/humble/setup.bash
```
for all the ROS stuff

### Join the exist container
```
$ ./hyundai_docker_join.sh
```

Useful alias for your _**host(local)**_ PC's .bashrc
```
alias amr_enter='cd ~/amr_ws/src/hyundai_amr/operations/scripts && bash hyundai_docker_join.sh'
```
Enjoy!

Connect docker and GUI env
```
$ xhost +local:docker
```

## ROS2 BUILD
```
$ colcon build --packages-up-to
$ colcon build --packages-select
```
Using alias
```
$ cbulid
```
This alias is for building ouster driver pkg only
```
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```


## System structure
Files should be placed as the following folder structure:
```
amr_ws
├── src
│   ├──hyundai_amr  
│   │   ├── common
│   │   ├── communication
│   │   ├── control
│   │   ├── perception
│   │   ├── planning
|   |   ├── sensor
|   |   ├── tools
|   |   ├── operations
├── build
├── install
```
<!-- 
## IP Setup
* Ouster os1-32ch: 192.168.1.20 &nbsp;
  (https://ouster.atlassian.net/servicedesk/customer/portal/8/topic/7b54ef32-342f-44bd-a04a-7be21e5084cd/article/775422070) -->

