# Buckeye Vertical Software 2024-25

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
