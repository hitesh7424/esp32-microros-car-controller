# ESP32 MicroROS Car Controller

> A ROS2 workspace for controlling ESP32-based robotic cars using MicroROS communication

## 🚗 Project Overview

This project provides a complete ROS2 workspace for interfacing with ESP32 microcontrollers running MicroROS firmware. It includes both C++ and Python nodes for car control and communication through a MicroROS agent.

## 📦 Package Contents

- **esp32_car**: Main ROS2 package with mixed C++/Python support
- **micro_ros_agent**: MicroROS agent for ESP32 communication
- Launch files for easy startup
- Python controller scripts
- C++ hello world example

## 🛠️ Installation & Setup

## 🛠️ Installation & Setup

### Prerequisites
- ROS2 (Jazzy or later)
- Python 3.8+
- colcon build tools

### Quick Start
```bash
# Clone the repository
git clone https://github.com/<your-username>/esp32-microros-car-controller.git
cd esp32-microros-car-controller

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Package Creation Commands

### Create a C++ ROS2 package with Python support:
```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name hello esp32_car
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name hello esp32_car
```

## Building the Package
```bash
colcon build --symlink-install
colcon build --packages-select esp32_car
```

## Directory Structure
- `src/` → C++ source files
- `scripts/` → Python executable scripts
- `launch/` → Launch files
- `esp32_car/` → Python package modules

## Making Python Work in C++ ROS Environment

### 1. Package Configuration (package.xml)
Add Python dependencies to your `package.xml`:
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>ament_cmake_python</buildtool_depend>
<depend>rclpy</depend>
```

### 2. CMakeLists.txt Configuration
Add these lines to support Python in your C++ package:
```cmake
# Find Python dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/controller.py  
  DESTINATION lib/${PROJECT_NAME}
)
```

### 3. Python Script Requirements
For Python scripts to work as ROS2 executables:

1. **Add shebang line** at the top:
   ```python
   #!/usr/bin/env python3
   ```

2. **Make script executable**:
   ```bash
   chmod +x scripts/controller.py
   ```

3. **Use proper ROS2 structure** (optional but recommended):
   ```python
   #!/usr/bin/env python3
   
   import rclpy
   from rclpy.node import Node
   
   class ControllerNode(Node):
       def __init__(self):
           super().__init__('controller_node')
           self.get_logger().info('Controller started')
   
   def main(args=None):
       rclpy.init(args=args)
       controller = ControllerNode()
       
       try:
           rclpy.spin(controller)
       except KeyboardInterrupt:
           pass
       finally:
           controller.destroy_node()
           rclpy.shutdown()
   
   if __name__ == '__main__':
       main()
   ```

### 4. Running Commands

#### Check available executables:
```bash
source install/setup.bash
ros2 pkg executables esp32_car
```

#### Run C++ executable:
```bash
ros2 run esp32_car hello
```

#### Run Python executable:
```bash
ros2 run esp32_car controller.py
```

#### Launch micro_ros_agent:
```bash
ros2 launch esp32_car esp_agent.py
```

### 5. Build and Test Workflow
```bash
# 1. Build the package
colcon build --packages-select esp32_car

# 2. Source the workspace
source install/setup.bash

# 3. Verify executables are available
ros2 pkg executables esp32_car

# 4. Test Python script
ros2 run esp32_car controller.py

# 5. Test C++ executable
ros2 run esp32_car hello
```

## Troubleshooting

### Python executable not showing up?
1. Check shebang line: `#!/usr/bin/env python3`
2. Make script executable: `chmod +x scripts/controller.py`
3. Verify CMakeLists.txt has proper install commands
4. Rebuild package: `colcon build --packages-select esp32_car`

### Import errors?
1. Add missing dependencies to `package.xml`
2. Install Python packages if needed: `pip install <package>`
3. Check Python path in workspace setup

## 🤝 Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## 🔗 Related Projects

- [MicroROS](https://micro.ros.org/) - Micro-ROS framework
- [ESP32 MicroROS Examples](https://github.com/micro-ROS/micro_ros_espidf_component)

## 📞 Contact

- **Author**: Hitesh
- **Email**: hitesh164722@gmail.com
- **Project**: ESP32 MicroROS Car Controller