# robolubo – Claude Code Context

## Project Summary
ROS2 Humble robot project. Code is developed locally and deployed to a Jetson Nano (`jetson-lubo`).
RViz and debugging tools run on the PC; ROS2 nodes run on the Jetson.

---

## Repo Layout
```
robolubo/
├── src/robolubo/          # ROS2 package (ament_cmake + Python nodes)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/robolubo/  # C++ headers
│   ├── src/               # C++ source files
│   ├── robolubo/          # Python module (import robolubo)
│   ├── launch/            # Launch files (*.launch.py)
│   └── config/            # YAML parameter files
├── scripts/               # Dev workflow scripts (run from PC, never deployed)
├── zenoh/                 # Zenoh bridge configs (jetson.json5, desktop.json5)
└── .env                   # Local config (gitignored) — copy from .env.example
```

**Never commit**: `build/`, `install/`, `log/`, `.env`

---

## Daily Workflow
```bash
# 1. Edit code in src/

# 2. Deploy + build on Jetson
scripts/deploy_and_build.sh

# 3. Drop into a shell on the Jetson to launch nodes
scripts/ssh.sh
  → source ~/robolubo_ws/install/setup.bash
  → ROS_LOCALHOST_ONLY=1 ros2 launch robolubo <launch_file>

# 4. On PC — start Zenoh bridge (in a separate terminal)
scripts/zenoh_local.sh

# 5. On PC — start RViz (another terminal)
source /opt/ros/humble/setup.bash && rviz2
```

---

## Network Architecture
- Jetson runs ROS2 nodes with default Fast DDS
- PC runs RViz + ROS2 tools with default Fast DDS
- `zenoh-bridge-ros2dds` bridges both sides over TCP (no multicast required)
- Jetson = **router** on port 7447
- PC = **client** connecting to `$JETSON_HOST:7447`
- Jetson hostname: `jetson-lubo` / `jetson-lubo.local` (IP fallback: `192.168.4.124`)

---

## Environment (.env)
| Variable | Description |
|---|---|
| `JETSON_HOST` | Jetson IP or hostname |
| `JETSON_USER` | SSH user on Jetson (alexis) |
| `JETSON_WORKSPACE` | Workspace dir on Jetson (robolubo_ws) |
| `ROS_DOMAIN_ID` | Must match on PC and Jetson |

---

## ROS2 Package: robolubo
- **Build type**: `ament_cmake` with `ament_cmake_python`
- **C++ nodes**: Add source to `src/`, register in `CMakeLists.txt` as `add_executable`
- **Python nodes**: Add to `robolubo/`, install via `ament_python_install_package`, register scripts with `install(PROGRAMS ...)`
- **Launch files**: `launch/*.launch.py`
- **Params**: `config/*.yaml`

### Adding a C++ node
1. Create `src/my_node.cpp`
2. In `CMakeLists.txt`, uncomment and adapt the `add_executable` + `ament_target_dependencies` + `install` block

### Adding a Python node
1. Create `robolubo/my_node.py` with `def main(): ...`
2. In `CMakeLists.txt`, uncomment and adapt the `install(PROGRAMS ...)` block
3. In `package.xml`, add `<exec_depend>` for any new Python packages

---

## Scripts Reference
| Script | What it does |
|---|---|
| `scripts/deploy.sh` | rsync `src/` → Jetson |
| `scripts/build.sh` | colcon build on Jetson via SSH |
| `scripts/deploy_and_build.sh` | deploy + build in one shot |
| `scripts/ssh.sh` | SSH into Jetson |
| `scripts/setup_jetson.sh` | **One-time**: set hostname, install zenoh bridge |
| `scripts/zenoh_jetson.sh` | Start zenoh router on Jetson |
| `scripts/zenoh_local.sh` | Start zenoh client on PC |

---

## One-time Setup (run once after cloning)
```bash
cp .env.example .env           # fill in JETSON_HOST etc.
chmod +x scripts/*.sh
scripts/setup_jetson.sh        # sets hostname, installs zenoh on Jetson
```
