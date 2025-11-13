# Duck Walk ROS 2 bring-up

This branch exposes the legacy `scripts/walk_test.py` runtime as a ROS 2 Humble
package named `duck_walk_bringup`. Launching

```
ros2 launch duck_walk_bringup walk.launch.py
```

is equivalent to running the original script, but now it can be chained with
other ROS 2 nodes, configured via launch arguments, and started from tooling
such as `ros2 launch`, `ros2 run`, or Docker.

## Repository layout

| Path | Purpose |
|------|---------|
| `duck_walk_bringup/` | ROS 2 package that exposes the walk test through launch. |
| `scripts/walk_test.py` | Original Python entry point that talks to the robot hardware. |
| `mini_bdx_runtime/` | Supporting Python modules reused by the entry point. |
| `Dockerfile` | Optional container image that ships ROS 2 Humble and the package pre-built. |

## Desktop / development host workflow

1. Install Ubuntu 22.04 with the ROS 2 Humble repositories enabled, then install
   the base dependencies:
   ```bash
   sudo apt update && sudo apt install \
       ros-humble-ros-base \
       python3-colcon-common-extensions \
       python3-argcomplete \
       python3-pip \
       git
   ```
2. Clone this branch and install the Python pieces in editable mode so the ROS 2
   package can import them:
   ```bash
   git clone https://github.com/apirrone/Open_Duck_Mini_Runtime.git
   cd Open_Duck_Mini_Runtime
   git checkout ros2_humble_pkg
   pip install -e .
   ```
3. Build the ROS 2 package and source the workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select duck_walk_bringup
   source install/setup.bash
   ```
4. Launch the bring-up:
   ```bash
   ros2 launch duck_walk_bringup walk.launch.py \
       use_fake_controller:=false \
       config:=example_config.json
   ```
   All parameters exposed by `walk_test.py` are available as launch arguments,
   so you can override serial devices, config files, or controller options
   directly from the command line.

## Raspberry Pi Zero 2 W workflow

The Pi Zero 2 W can run the same ROS 2 package as long as it uses a 64-bit OS
with Humble binaries (Ubuntu Server 22.04 is the easiest option). After flashing
and booting the board:

1. Configure networking/SSH, then install ROS 2 Humble and build tools:
   ```bash
   sudo apt update && sudo apt install \
       ros-humble-ros-base \
       python3-colcon-common-extensions \
       python3-argcomplete \
       python3-pip \
       git
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```
2. Make sure the `pi` (or your user) account can access the hardware that the
   walk test expects:
   ```bash
   sudo usermod -a -G dialout,i2c $USER
   sudo raspi-config nonint do_i2c 0
   sudo udevadm control --reload
   ```
   Log out/in so the new group membership takes effect, then connect the FTDI
   adapter, IMU, servos, and controller as usual.
3. Clone the repository, install the Python runtime, and build the ROS 2 package:
   ```bash
   git clone https://github.com/apirrone/Open_Duck_Mini_Runtime.git
   cd Open_Duck_Mini_Runtime
   git checkout ros2_humble_pkg
   pip install -e .
   colcon build --packages-select duck_walk_bringup
   source install/setup.bash
   ```
4. Run the launch file just like on the desktop:
   ```bash
   ros2 launch duck_walk_bringup walk.launch.py \
       config:=example_config.json \
       motor_port:=/dev/ttyUSB0 \
       controller_name:=xbox
   ```
   Use `use_fake_controller:=true` if you need to validate the robot without a
   paired gamepad, or override any other launch argument to point to different
   hardware paths.

## Optional Docker workflow

If you prefer not to install ROS 2 on the host, build the provided image and
run the launch file inside the container:

```bash
docker build -t duck-mini-runtime:humble .
docker run --rm -it \
    --net=host \
    --privileged \
    --device /dev/i2c-1 \
    --device /dev/ttyUSB0 \
    duck-mini-runtime:humble
```

The container automatically sources both `/opt/ros/humble` and the local
workspace, so you can immediately execute `ros2 launch duck_walk_bringup
walk.launch.py` after attaching to the shell. Add or remove `--device` flags to
match the hardware connected to your machine.
