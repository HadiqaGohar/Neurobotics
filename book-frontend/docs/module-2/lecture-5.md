---
sidebar_position: 5
---

# Lecture 5: Building ROS 2 Packages

## What is a ROS 2 Package?

A ROS 2 package is a collection of related files organized in a standardized way. It's the fundamental unit for organizing and distributing ROS 2 code. Think of it as a container that holds:

- **Source code** (Python or C++)
- **Configuration files** (parameters, launch files)
- **Dependencies** (other packages it needs)
- **Metadata** (package information)

## Package Types

### 1. Python Packages
- Use `setuptools` for building
- Source code in Python
- Good for rapid prototyping and AI integration

### 2. C++ Packages  
- Use `CMake` for building
- Source code in C++
- Better performance for real-time applications

### 3. Hybrid Packages
- Contain both Python and C++ code
- More complex but very flexible

## Creating Your First Package

### Step 1: Set Up Workspace

```bash
# Create workspace directory
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Initialize workspace (if not done already)
cd ~/robot_ws
colcon build
source install/setup.bash
```

### Step 2: Create Package

```bash
# Navigate to src directory
cd ~/robot_ws/src

# Create Python package
ros2 pkg create --build-type ament_python my_robot_package

# Or create C++ package
ros2 pkg create --build-type ament_cmake my_robot_cpp_package

# Create package with dependencies
ros2 pkg create --build-type ament_python my_robot_package \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

## Python Package Structure

```
my_robot_package/
├── my_robot_package/           # Python module directory
│   ├── __init__.py            # Makes it a Python package
│   ├── robot_controller.py    # Main robot logic
│   ├── sensor_processor.py    # Sensor data processing
│   └── utils.py              # Utility functions
├── resource/                  # Package marker
│   └── my_robot_package       # Empty file
├── test/                      # Unit tests
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── launch/                    # Launch files (create manually)
│   └── robot_launch.py
├── config/                    # Configuration files (create manually)
│   └── robot_params.yaml
├── package.xml               # Package metadata
├── setup.py                  # Python package setup
└── setup.cfg                 # Python configuration
```

## Package.xml - Package Metadata

The `package.xml` file describes your package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <!-- Basic package information -->
  <name>my_robot_package</name>
  <version>1.0.0</version>
  <description>A comprehensive robot control package</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>
  
  <!-- URLs for more information -->
  <url type="website">https://github.com/yourusername/my_robot_package</url>
  <url type="bugtracker">https://github.com/yourusername/my_robot_package/issues</url>
  <url type="repository">https://github.com/yourusername/my_robot_package</url>
  
  <!-- Build dependencies (needed to compile) -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- Runtime dependencies (needed to run) -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  
  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>python3-pytest</test_depend>
  
  <!-- Export information -->
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Setup.py - Python Package Configuration

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Install package marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        
        # Install URDF files (if any)
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        
        # Install mesh files (if any)
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A comprehensive robot control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Executable name = package.module:function
            'robot_controller = my_robot_package.robot_controller:main',
            'sensor_processor = my_robot_package.sensor_processor:main',
            'camera_node = my_robot_package.camera_node:main',
            'lidar_node = my_robot_package.lidar_node:main',
            'navigation_node = my_robot_package.navigation_node:main',
        ],
    },
)
```

## Creating Executable Nodes

### Main Robot Controller Node

**File: `my_robot_package/robot_controller.py`**

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

class RobotController(Node):
    """Main robot controller that coordinates all robot functions"""
    
    def __init__(self):
        super().__init__('robot_controller')
        
        # Declare parameters
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('robot_name', 'robot')
        
        # Get parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.robot_name = self.get_parameter('robot_name').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            f'/{self.robot_name}/cmd_vel', 
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            f'/{self.robot_name}/status',
            10
        )
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.laser_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )
        
        # Robot state
        self.current_pose = None
        self.obstacle_detected = False
        self.min_distance = float('inf')
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(f'Robot Controller for {self.robot_name} started')
    
    def laser_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        # Find minimum distance in front of robot (±30 degrees)
        front_angles = len(msg.ranges) // 6  # ±30 degrees
        start_idx = len(msg.ranges) // 2 - front_angles
        end_idx = len(msg.ranges) // 2 + front_angles
        
        front_ranges = msg.ranges[start_idx:end_idx]
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]
        
        if valid_ranges:
            self.min_distance = min(valid_ranges)
            self.obstacle_detected = self.min_distance < self.safety_distance
        else:
            self.min_distance = float('inf')
            self.obstacle_detected = False
    
    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.current_pose = msg.pose.pose
    
    def control_loop(self):
        """Main control loop - implement your robot behavior here"""
        cmd = Twist()
        
        if self.obstacle_detected:
            # Stop if obstacle detected
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn to avoid obstacle
            self.get_logger().warn(f'Obstacle detected at {self.min_distance:.2f}m')
        else:
            # Simple forward movement (replace with your logic)
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
        
        # Apply speed limits
        cmd.linear.x = max(-self.max_linear_speed, 
                          min(self.max_linear_speed, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular_speed,
                           min(self.max_angular_speed, cmd.angular.z))
        
        self.cmd_vel_pub.publish(cmd)
    
    def publish_status(self):
        """Publish robot status information"""
        status_msg = String()
        
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            status = "OBSTACLE_AVOIDANCE" if self.obstacle_detected else "NORMAL"
            status_msg.data = f"Robot: {self.robot_name}, Position: ({x:.2f}, {y:.2f}), Status: {status}"
        else:
            status_msg.data = f"Robot: {self.robot_name}, Status: INITIALIZING"
        
        self.status_pub.publish(status_msg)

def main(args=None):
    """Main function to run the robot controller"""
    rclpy.init(args=args)
    
    try:
        robot_controller = RobotController()
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'robot_controller' in locals():
            robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Sensor Processing Node

**File: `my_robot_package/sensor_processor.py`**

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class SensorProcessor(Node):
    """Process and filter sensor data"""
    
    def __init__(self):
        super().__init__('sensor_processor')
        
        # Parameters
        self.declare_parameter('filter_window_size', 5)
        self.declare_parameter('max_range', 10.0)
        
        self.filter_size = self.get_parameter('filter_window_size').value
        self.max_range = self.get_parameter('max_range').value
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # Publishers
        self.filtered_laser_pub = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            10
        )
        
        self.features_pub = self.create_publisher(
            Float32MultiArray,
            '/laser_features',
            10
        )
        
        # Data storage for filtering
        self.laser_history = []
        
        self.get_logger().info('Sensor Processor started')
    
    def laser_callback(self, msg):
        """Process laser scan data"""
        # Store for temporal filtering
        self.laser_history.append(msg.ranges)
        if len(self.laser_history) > self.filter_size:
            self.laser_history.pop(0)
        
        # Apply filtering
        filtered_msg = self.filter_laser_scan(msg)
        self.filtered_laser_pub.publish(filtered_msg)
        
        # Extract features
        features = self.extract_laser_features(filtered_msg)
        self.features_pub.publish(features)
    
    def filter_laser_scan(self, msg):
        """Apply temporal and spatial filtering to laser data"""
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        
        # Temporal filtering (average over recent scans)
        if len(self.laser_history) >= self.filter_size:
            ranges_array = np.array(self.laser_history)
            filtered_ranges = np.mean(ranges_array, axis=0)
        else:
            filtered_ranges = np.array(msg.ranges)
        
        # Spatial filtering (median filter)
        filtered_ranges = self.median_filter(filtered_ranges, window=3)
        
        # Remove outliers
        filtered_ranges = np.clip(filtered_ranges, msg.range_min, self.max_range)
        
        filtered_msg.ranges = filtered_ranges.tolist()
        filtered_msg.intensities = msg.intensities
        
        return filtered_msg
    
    def median_filter(self, data, window=3):
        """Apply median filter to remove noise"""
        filtered = np.copy(data)
        half_window = window // 2
        
        for i in range(half_window, len(data) - half_window):
            window_data = data[i-half_window:i+half_window+1]
            filtered[i] = np.median(window_data)
        
        return filtered
    
    def extract_laser_features(self, msg):
        """Extract useful features from laser scan"""
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]
        
        if len(valid_ranges) == 0:
            features = [0.0] * 6  # Return zeros if no valid data
        else:
            features = [
                float(np.min(valid_ranges)),      # Minimum distance
                float(np.max(valid_ranges)),      # Maximum distance
                float(np.mean(valid_ranges)),     # Average distance
                float(np.std(valid_ranges)),      # Standard deviation
                float(len(valid_ranges)),         # Number of valid points
                float(np.sum(valid_ranges < 1.0)) # Number of close obstacles
            ]
        
        # Create message
        feature_msg = Float32MultiArray()
        feature_msg.data = features
        
        return feature_msg

def main(args=None):
    rclpy.init(args=args)
    
    try:
        sensor_processor = SensorProcessor()
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'sensor_processor' in locals():
            sensor_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Building and Installing Packages

### Build Process

```bash
# Navigate to workspace root
cd ~/robot_ws

# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_robot_package

# Build with verbose output
colcon build --event-handlers console_direct+

# Build in parallel (faster)
colcon build --parallel-workers 4
```

### Source the Workspace

```bash
# Source the workspace (do this after every build)
source ~/robot_ws/install/setup.bash

# Add to bashrc for automatic sourcing
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
```

### Test Your Package

```bash
# List available executables
ros2 pkg executables my_robot_package

# Run your nodes
ros2 run my_robot_package robot_controller

# Run with parameters
ros2 run my_robot_package robot_controller --ros-args -p max_linear_speed:=2.0

# Check if package is properly installed
ros2 pkg list | grep my_robot_package
```

## Package Dependencies

### Adding Dependencies

**In package.xml:**
```xml
<!-- Add new dependencies -->
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<depend>tf2_ros</depend>
```

**In setup.py:**
```python
install_requires=[
    'setuptools',
    'opencv-python',  # Python-specific dependencies
    'numpy',
    'scipy'
]
```

### Dependency Types

1. **Build dependencies**: Needed to compile the package
2. **Runtime dependencies**: Needed to run the package
3. **Test dependencies**: Needed for testing

## Custom Messages and Services

### Creating Custom Messages

**Create message directory:**
```bash
mkdir -p my_robot_package/msg
```

**File: `my_robot_package/msg/RobotStatus.msg`**
```
# Custom message for robot status
string robot_id
float64 battery_level
geometry_msgs/Pose current_pose
bool is_autonomous
string[] active_sensors
builtin_interfaces/Time timestamp
```

**Update package.xml:**
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**Update setup.py:**
```python
# Add to setup.py
from setuptools import setup
from glob import glob
import os

setup(
    # ... existing setup ...
    data_files=[
        # ... existing data_files ...
        # Install message files
        (os.path.join('share', package_name, 'msg'),
            glob('msg/*.msg')),
    ],
)
```

## Testing Your Package

### Unit Tests

**File: `test/test_robot_controller.py`**
```python
import unittest
import rclpy
from my_robot_package.robot_controller import RobotController

class TestRobotController(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = RobotController()
    
    def tearDown(self):
        self.node.destroy_node()
    
    def test_parameter_initialization(self):
        """Test that parameters are properly initialized"""
        self.assertIsNotNone(self.node.max_linear_speed)
        self.assertGreater(self.node.max_linear_speed, 0)
    
    def test_safety_distance(self):
        """Test safety distance parameter"""
        self.assertGreater(self.node.safety_distance, 0)
        self.assertLess(self.node.safety_distance, 10)

if __name__ == '__main__':
    unittest.main()
```

### Run Tests

```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select my_robot_package

# Show test results
colcon test-result --verbose
```

## Best Practices

### Code Organization
1. **One node per file**: Keep nodes focused and manageable
2. **Use meaningful names**: Clear function and variable names
3. **Add docstrings**: Document all classes and functions
4. **Handle errors gracefully**: Use try-catch blocks

### Package Structure
1. **Logical grouping**: Group related functionality
2. **Clear dependencies**: Only depend on what you need
3. **Version control**: Use semantic versioning
4. **Documentation**: Include README and examples

### Performance
1. **Efficient callbacks**: Keep callback functions fast
2. **Appropriate rates**: Don't publish faster than needed
3. **Memory management**: Clean up resources properly
4. **Profiling**: Monitor CPU and memory usage

## Common Issues and Solutions

### Build Errors
```bash
# Clean build if having issues
rm -rf build install log
colcon build

# Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Import Errors
```bash
# Make sure workspace is sourced
source install/setup.bash

# Check if package is in Python path
python3 -c "import my_robot_package; print('Success')"
```

### Node Not Found
```bash
# Verify executable is registered
ros2 pkg executables my_robot_package

# Check setup.py entry_points
```

## Key Takeaways

- ROS 2 packages organize related code and resources
- Package.xml defines metadata and dependencies
- Setup.py configures Python packages and executables
- Proper structure makes packages maintainable and reusable
- Testing ensures code quality and reliability
- Following conventions makes packages easier to use and share
- Build system handles compilation and installation automatically

---

**Congratulations!** You've completed Chapter 2 and learned the fundamentals of ROS 2. You now understand nodes, communication, launch files, and package creation.

**Next Chapter:** [Chapter 3: Robot Simulation and Digital Twins](../chapter-3/lecture-1.md)