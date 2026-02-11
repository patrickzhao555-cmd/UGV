# UGV
# RTX_UGV — Fresh Setup + Run Guide (Jetson / Ubuntu 22.04 + ROS 2 Humble + ZED 2i)

This README explains how to go from a **fresh Jetson system** to running:
- ZED 2i camera bring-up (RGB + depth topics)
- ZED built-in Object Detection (optional)
- Our depth-based obstacle warning node
- Presentation-friendly live view (raw + overlay boxes/labels/distance)

---

## 0) Assumptions
- **OS:** Ubuntu **22.04 LTS** (JetPack 6 / R36.x is OK)
- **ROS 2:** Humble
- **Camera:** ZED 2i (USB 3.0 recommended)
- Repo contains: `ros2_ws/src` with ZED ROS2 packages + our `ugv_*` packages

Check system:
```bash
lsb_release -a
cat /etc/nv_tegra_release
