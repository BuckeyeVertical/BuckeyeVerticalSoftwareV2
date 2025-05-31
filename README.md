# Buckeye Vertical Software 2024-25

## Setting up the repo:
Step 1: Clone the repo
```
git clone https://github.com/BuckeyeVertical/BuckeyeVerticalSoftwareV2.git
```
or if you prefer ssh:
```
git@github.com:BuckeyeVertical/BuckeyeVerticalSoftwareV2.git
```

Step 2: Switch the version
On the Jetson/Raspi use SNAPSHOT-1.0.0.
For development use DEV-1.0.0
```
cd BuckeyeVerticalSoftwareV2
git checkout SNAPSHOT-1.0.0
```

Step 3: **On the Jetson/Raspi only!** Set up COLCON_IGNORE
```
cd src/ros_gz_bridge
touch COLCON_IGNORE
```

## Building the docker:
Step 1:
```
cd bv_ws/BuckeyeVerticalSoftwareV2/containers/raspi
```

Step 2: Build the container
```
docker build -t "CONTAINER_NAME" .
```

## Running the code on the raspi:
Step 1:
```
cd bv_ws/BuckeyeVerticalSoftwareV2/containers/raspi
```

Step 2:
Specify the launch file to run in containers/raspi/launch.sh by setting the LAUNCH_FILE variable

Step 3:
Run the launch file:
```
./run_docker.sh
```

## Debugging:
To run the docker in bash mode (don't run anything automatically):
```
./run_docker_bash.sh
```

To shell into already running docker:
```
docker exec -it dreamer_container /bin/bash
```

To check pixhawk <-> raspi connection run this (inside container):
```
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```
If the bridge is working, then all the topics should be created.
