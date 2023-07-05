# TII_multidrone_base
Dockerized Multi-Aerial Vehicle Simulator for TII Project

### Build the dockerfile and launch simulation environment
Note that python3 needs to be installed to use an automatic build and a launch script.

1. To build the dockerfile for the first time, 

```bash
python3 auto_generate_scripts.py -n (number_of_drones) -i (your_indicator) -p (network_address_offset)
```

where '(number_of_drones)' denotes the number of drones that you want to launch in a simulation and '(your_indicator)' and '(network_address_offset)' arguments stand for your the suffix to indicate your own container name and the network address offset for your container network. '-i' and '-p' options can be used for the different users in a server running this environments. This is to prevent a container from network and container name overlap.

2. To launch the simulation automatically,

```bash
python3 run_docker_simulation.py
```
you can see the terminators that runs each drone's PX4 autopilot and uXRCE-DDS Agent. Gazebo Garden with the drones will start as well.

### Note
'pre-run.sh' is required to setup the environment if you manually launch the docker compose file. It sets your GID/UID as environment variables for the container and enables the communication between containers and X windows in the host.