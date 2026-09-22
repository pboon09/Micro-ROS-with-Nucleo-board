# Install

Needs ROS 2 Humble or Jazzy, already installed and sourced.

## 1. micro-ROS agent

```bash
mkdir microros_ws
cd microros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

colcon build
source install/local_setup.bash

ros2 run micro_ros_setup create_firmware_ws.sh host
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

## 2. Docker

```bash
sudo apt install -y docker.io
sudo usermod -aG docker,dialout $USER
```

`dialout` lets the agent open `/dev/ttyACM0`. Log out and back in, then check it. Expect `Hello from Docker!`.

```bash
docker run hello-world
```

## 3. STM32CubeIDE

Download **STM32CubeIDE Debian Linux Installer** from [st.com](https://www.st.com/en/development-tools/stm32cubeide.html) into `~/Downloads`. It needs a free ST account.

`yes |` accepts the license prompts for you.

```bash
cd ~/Downloads && unzip en.st-stm32cubeide_*_amd64.deb_bundle.sh.zip
chmod +x st-stm32cubeide_*_amd64.deb_bundle.sh && yes | sudo sh ./st-stm32cubeide_*_amd64.deb_bundle.sh
```

Check it. Expect a `stm32cubeide_<version>` folder.

```bash
ls /opt/st/
```
