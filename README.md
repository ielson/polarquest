# Polaris GEM – System ID + MPC (ROS Noetic, Docker)

Minimal ROS stack to simulate the Polaris GEM vehicle, publish reference paths from CSV, and run a simple MPC controller (steering-only or steering+speed). Everything runs inside a Docker container.

---

## Contents

- **`gem_gazebo`** – Polaris GEM Gazebo + RViz launcher (slightly customized UI)
- **`polaris_system_id`**
  - `mpc.py` – MPC controller (`steer_only` or `speed_steer`)
  - `publish_from_csv.py` – publishes a path from a CSV file
  - `odom_to_tf.py` – bridges Odometry → TF

---

## 1) Install Docker (Ubuntu)

> These steps follow the official docs:  
> Docker: <https://docs.docker.com/engine/install/ubuntu/>  
> NVIDIA Toolkit: <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html>

```bash
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
```
Test the service:
```bash
sudo systemctl status docker
```
If it’s active, run the “hello world”:
```bash
sudo docker run hello-world
```
Run Docker without sudo:
```bash
sudo usermod -aG docker $USER
newgrp docker
```


## Enable NVIDIA GPU in Containers

```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
   curl \
   gnupg2
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.0-1
sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
Verify GPU access
```bash
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu20.04 nvidia-smi
```
If you see the nvidia-smi table, GPU is working in Docker.

## 3) Start the Docker Environment

From the project folder that contains the docker-compose.yml:
```bash
cd <your-download-folder>/docker
docker compose up -d noetic
```
On the first start, the container build/start scripts will install all dependencies, clone required repos, run catkin_make, and source the devel/setup.bash automatically.

Open a shell (repeat for as many terminals as you need):
```bash
docker exec -it -w /home/dev/code/polarquest/ws noetic-dev bash
```

## 4) Run the Simulation and Nodes
### Launch Gazebo + RViz
```bash
roslaunch gem_gazebo gem_gazebo_rviz.launch
```
This starts the Polaris GEM simulation with a customized interface.

### Publish TF from Odometry
```bash
rosrun polaris_system_id odom_to_tf.py
```

### MPC Controller

Steering-only (constant speed):
```bash
rosrun polaris_system_id mpc.py _mode:=steer_only _v_fixed:=6.0 _Ts:=0.05 _Np:=20 _solve_every:=3
```

Steering + speed:
```bash
rosrun polaris_system_id mpc.py _mode:=speed_steer _Ts:=0.05 _Np:=20 _solve_every:=3 _spd_min:=0.5 _spd_max:=8.0 _spd_slew:=0.6
```
Key params:=
- _mode: steer_only or speed_steer
- _v_fixed: constant speed for steer_only (m/s)
- _Ts: controller sample time (s)
- _Np: prediction horizon (steps)
- _solve_every: re-solve interval (every N steps)
- _spd_min/_spd_max: speed bounds (m/s)
- _spd_slew: max speed change per step (m/s/step)

### Publish a Path from CSV
```bash
rosrun polaris_system_id publish_from_csv.py
```
Publishes a path topic consumed by the MPC. 

