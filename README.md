# Open Duck Mini Runtime

## Raspberry Pi zero 2W setup

### Install Raspberry Pi OS

Download Raspberry Pi OS Lite (64-bit) from here : https://www.raspberrypi.com/software/operating-systems/

Follow the instructions here to install the OS on the SD card : https://www.raspberrypi.com/documentation/computers/getting-started.html

With the Raspberry Pi Imager, you can pre-configure session, wifi and ssh. Do it like below :

![imager_setup](https://github.com/user-attachments/assets/7a4987b2-de83-41dd-ab7f-585259685f16)

> Tip: I configure the rasp to connect to my phone's hotspot, this way I can connect to it from anywhere.

### Setup SSH (If not setup during the installation)

When first booting on the rasp, you will need to connect a screen and a keyboard. The first thing you should do is connect to a wifi network and enable SSH.

To do so, you can follow this guide : https://www.raspberrypi.com/documentation/computers/configuration.html#setting-up-wifi

Then, you can connect to your rasp using SSH without having to plug a screen and a keyboard.

### Update the system and install necessary stuff

```bash
sudo apt update
sudo apt upgrade
sudo apt install git
sudo apt install python3-pip
sudo apt install python3-virtualenvwrapper
(optional) sudo apt install python3-picamzero

```

Add this to the end of the `.bashrc`:

```bash
export WORKON_HOME=$HOME/.virtualenvs
export PROJECT_HOME=$HOME/Devel
source /usr/share/virtualenvwrapper/virtualenvwrapper.sh
```

### Enable I2C

`sudo raspi-config` -> `Interface Options` -> `I2C`

TODO set 400KHz ?

### Set the usbserial latency timer

```bash
cd  /etc/udev/rules.d/
sudo touch 99-usb-serial.rules
sudo nano 99-usb-serial.rules
# copy the following line in the file
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

### Set the udev rules for the motor control board

TODO


### Setup xbox one controller over bluetooth

Turn your xbox one controller on and set it in pairing mode by long pressing the sync button on the top of the controller.

Run the following commands on the rasp :

```bash
bluetoothctl
scan on
```

Wait for the controller to appear in the list, then run :

```bash
pair <controller_mac_address>
trust <controller_mac_address>
connect <controller_mac_address>
```

The led on the controller should stop blinking and stay on.

You can test that it's working by running

```bash
python3 mini_bdx_runtime/mini_bdx_runtime/xbox_controller.py
```

## Speaker wiring and configuration
Follow this tutorial

> For now, don't activate `/dev/zero` when they ask

https://learn.adafruit.com/adafruit-max98357-i2s-class-d-mono-amp?view=all


## Install the runtime

### Make a virtual environment and activate it

```bash
mkvirtualenv -p python3 open-duck-mini-runtime
workon open-duck-mini-runtime
```

Clone this repository on your rasp, cd into the repo, then :

```bash
git clone https://github.com/apirrone/Open_Duck_Mini_Runtime
cd Open_Duck_Mini_Runtime
git checkout v2
pip install -e .
```

## ROS 2 Humble bring-up branch

To keep the existing Python runtime intact while experimenting with ROS 2, a
dedicated branch named `ros2_humble_pkg` now hosts the ROS 2 package and launch
assets. Switch to that branch to access the `duck_walk_bringup` package:

```bash
git checkout ros2_humble_pkg
colcon build --packages-select duck_walk_bringup
source install/setup.bash
ros2 launch duck_walk_bringup walk.launch.py
```

Running the launch file mirrors the original `scripts/walk_test.py` behaviour
but exposes all parameters through ROS 2 launch arguments so it can be chained
with other ROS 2 nodes.

In Raspberry Pi 5, you need to perform the following operations

```bash
pip uninstall -y RPi.GPIO
pip install lgpio
```

## Docker workspace (ROS 2 + Python runtime)

If you prefer to avoid installing ROS 2 Humble and the Python runtime on the
host machine, you can build a Docker image that already contains both the
`mini_bdx_runtime` package and the `duck_walk_bringup` ROS 2 launch file. The
image is based on `ros:humble-ros-base`, so it behaves the same as a native ROS
2 shell once started.

### Build the image

```bash
docker build -t duck-mini-runtime:humble .
```

The build stage installs all Python dependencies, registers the
`mini_bdx_runtime` package in editable mode, and runs `colcon build` for the
`duck_walk_bringup` ROS 2 package, so the container is ready to launch as soon
as it starts.

### Run the container

```bash
docker run --rm -it \
    --net=host \
    --privileged \
    --device /dev/i2c-1 \
    --device /dev/ttyUSB0 \
    duck-mini-runtime:humble
```

* Use `--net=host` if you need ROS 2 DDS discovery with other machines.
* Add `--device` (or `-v`) flags for any hardware you want to expose
  (controllers, IMU, speaker amp, etc.). The example above exposes the default
  I²C bus and a USB serial adapter.
* Mount a local folder if you want to keep logs or configuration outside the
  container, e.g. `-v $(pwd)/data:/data`.

Once inside the shell the ROS 2 environment is sourced automatically, so you
can run the walk test via launch:

```bash
ros2 launch duck_walk_bringup walk.launch.py
```

or execute the legacy Python script directly:

```bash
python3 scripts/walk_test.py
```


## Test the IMU

```bash
python3 mini_bdx_runtime/mini_bdx_runtime/raw_imu.py
```

You can also run `python3 scripts/imu_server.py` on the robot and `python3 scripts/imu_client.py --ip <robot_ip>` on your computer to check that the frame is oriented correctly. 

> To find the ip address of the robot, run `ifconfig` on the robot

## Test motors

This will allow you to verify all your motors are connected and configured.

```bash
python3 scripts/check_motors.py
```

## Make your duck_config.json

Copy `example_config.json` in the home directory of your duck and rename it `duck_config.json`.

`cp example_config.json ~/duck_config.json`

In this file, you can configure some stuff, like registering if you installed the expression features, installed the imu upside down or and other stuff. You also write the joints offsets of your duck here

## Find the joints offsets

This script will guide you through finding the joints offsets of your robot that you can then write in your `duck_config.json`

> This procedure won't be necessary in the future as we will be flashing the offsets directly in each motor's eeprom.

```bash
cd scripts/
python find_soft_offsets.py
```

## Run the walk !

Download the [latest policy checkpoint ](https://github.com/apirrone/Open_Duck_Mini/blob/v2/BEST_WALK_ONNX_2.onnx) and copy it to your duck.

`cd scripts/`

`python v2_rl_walk_mujoco.py --onnx_model_path <path_to>/BEST_WALK_ONNX_2.onnx`



```
- The commands are : 
- A to pause/unpause
- X to turn on/off the projector
- B to play a random sound
- Y to turn on/off head control (very experimental, I don't recommend trying that, it can break your duck's head)
- left and right triggers to control the left and right antennas
- LB (new!) press and hold to increase the walking frequency, kind of a sprint mode 🙂
```