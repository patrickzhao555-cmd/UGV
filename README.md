# UGV
## RTX_UGV — Fresh Setup + Run Guide (Jetson / Ubuntu 22.04 + ROS2 Humble)

### 0) System assumptions

* **OS:** Ubuntu **22.04 LTS** (JetPack 6 / R36.x is OK)
* **ROS2:** Humble
* **Main camera:** ZED 2i (USB)
* Repo contains: `ros2_ws/src` with ZED ROS2 packages + our `ugv_*` packages.

Check system:

```bash
lsb_release -a
cat /etc/nv_tegra_release
```

---

# 1) Install base tools

```bash
sudo apt update
sudo apt install -y git curl gnupg lsb-release build-essential cmake python3-pip python3-venv
```

**Important (ROS + cv_bridge stability):**
Avoid installing NumPy/OpenCV via pip system-wide. Use apt packages instead:

```bash
sudo apt install -y python3-numpy python3-opencv
```

---

# 2) Install ROS 2 Humble (Ubuntu 22.04)

If ROS2 is not installed yet:

```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-rosdep
```

Initialize rosdep:

```bash
sudo rosdep init || true
rosdep update
```

Auto-source ROS2:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# 3) Install ZED SDK (required)

ZED ROS2 wrapper requires the **ZED SDK** installed on the Jetson.

* Go to **Stereolabs ZED SDK download page**
* Download the **Jetson / JetPack (ARM64)** version that matches your JetPack (R36.x)
* Install it (usually a `.run` installer)

After install, verify:

```bash
ls /usr/local/zed
```

---

# 4) Clone this repo

```bash
cd ~
git clone https://github.com/patrickzhao555-cmd/RTX_UGV.git
cd RTX_UGV
```

(If you already have `ugv_project`, just use that folder instead.)

---

# 5) Install ROS dependencies via rosdep

```bash
cd ~/RTX_UGV/ros2_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
```

If you ever see cv_bridge / numpy errors, **do NOT** upgrade numpy via pip. Keep:

* `numpy 1.x` (Ubuntu apt)
* OpenCV via apt
* cv_bridge via ROS apt

Sanity test:

```bash
python3 -c "import numpy; import cv2; from cv_bridge import CvBridge; print('numpy', numpy.__version__); print('cv2', cv2.__version__); print('cv_bridge OK')"
```

---

# 6) Build the workspace

```bash
cd ~/RTX_UGV/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

Auto-source this workspace (optional, but convenient):

```bash
echo "source ~/RTX_UGV/ros2_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# 7) Bring-up: Run ZED camera (Terminal 1)

Plug in ZED 2i (USB3 recommended), then:

```bash
source /opt/ros/humble/setup.bash
source ~/RTX_UGV/ros2_ws/install/local_setup.bash

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

Verify topics:

```bash
ros2 topic list | grep zed
```

---

# 8) Enable Object Detection (optional) (Terminal 2)

If you want ZED built-in OD:

```bash
source /opt/ros/humble/setup.bash
source ~/RTX_UGV/ros2_ws/install/local_setup.bash

ros2 service call /zed/zed_node/enable_obj_det std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /zed/zed_node/obj_det/objects --once
```

---

# 9) Run obstacle warning (depth-based) (Terminal 3)

This publishes:

* `/ugv/obstacle_distance_m`
* `/ugv/obstacle_warning`

```bash
source /opt/ros/humble/setup.bash
source ~/RTX_UGV/ros2_ws/install/local_setup.bash

ros2 run ugv_perception obstacle_warning
```

In another terminal, monitor:

```bash
ros2 topic echo /ugv/obstacle_distance_m
ros2 topic echo /ugv/obstacle_warning
```

---

# 10) Live camera view (presentation)

## Option A: Raw image view

Install viewer:

```bash
sudo apt install -y ros-humble-rqt-image-view
```

Run:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select topic:

* `/zed/zed_node/rgb/color/rect/image`

## Option B: Overlay view (boxes + labels + distance)

Run overlay node:

```bash
ros2 run ugv_perception zed_overlay
```

Then open rqt:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

* `/ugv/zed/overlay_image`

> Note: if you are SSH-only with no desktop, GUI windows won’t “pop up” on Jetson. Use VNC/desktop or run rqt on a GUI machine on the same ROS network.

---

# 11) Common issues

### “ros2 topic hz prints nothing”

Usually ZED isn’t running yet. Relaunch:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

### cv_bridge / numpy errors

If you see `_ARRAY_API not found` or `numpy.core.multiarray failed to import`:

* uninstall pip numpy:

```bash
python3 -m pip uninstall -y numpy opencv-python opencv-python-headless
```

* reinstall apt versions:

```bash
sudo apt install -y python3-numpy python3-opencv ros-humble-cv-bridge
```

---

