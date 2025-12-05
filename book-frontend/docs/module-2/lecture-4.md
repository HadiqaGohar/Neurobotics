---
sidebar_position: 4
---

# Lecture 4: Launch Files and Parameters

## What are Launch Files?

Launch files are configuration scripts that start multiple ROS 2 nodes with specific settings. Instead of manually starting each node in separate terminals, launch files automate the entire process.

### Why Use Launch Files?

**Without Launch Files:**
```bash
# Terminal 1
ros2 run my_robot camera_node

# Terminal 2  
ros2 run my_robot lidar_node

# Terminal 3
ros2 run my_robot navigation_node --ros-args -p max_speed:=2.0

# Terminal 4
ros2 run my_robot control_node --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

**With Launch Files:**
```bash
# Single command starts everything
ros2 launch my_robot robot_launch.py
```

## Basic Launch File Structure

Launch files in ROS 2 are Python scripts that use the launch system API:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='camera_node',
            name='camera'
        ),
        Node(
            package='my_robot', 
            executable='lidar_node',
            name='lidar'
        ),
        Node(
            package='my_robot',
            executable='navigation_node',
            name='navigation'
        )
    ])
```

## Complete Launch File Example

### Robot System Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot')
    
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='Whether to use simulation'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    
    # Configuration file paths
    config_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    
    # Launch configuration variables
    robot_name = LaunchConfiguration('robot_name')
    use_simulation = LaunchConfiguration('use_simulation')
    debug_mode = LaunchConfiguration('debug')
    
    return LaunchDescription([
        # Launch arguments
        robot_name_arg,
        use_sim_arg,
        debug_arg,
        
        # Log launch information
        LogInfo(
            msg=['Launching robot: ', robot_name]
        ),
        
        # Camera node
        Node(
            package='my_robot',
            executable='camera_node',
            name='camera',
            namespace=robot_name,
            parameters=[config_file],
            remappings=[
                ('/camera/image', [robot_name, '/camera/image'])
            ],
            condition=UnlessCondition(use_simulation)
        ),
        
        # Simulated camera (only in simulation)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='camera_sim',
            arguments=['-entity', 'camera', '-topic', 'robot_description'],
            condition=IfCondition(use_simulation)
        ),
        
        # LIDAR node
        Node(
            package='my_robot',
            executable='lidar_node',
            name='lidar',
            namespace=robot_name,
            parameters=[
                config_file,
                {'debug': debug_mode}
            ],
            output='screen' if debug_mode else 'log'
        ),
        
        # Navigation node
        Node(
            package='my_robot',
            executable='navigation_node',
            name='navigation',
            namespace=robot_name,
            parameters=[config_file],
            remappings=[
                ('/cmd_vel', [robot_name, '/cmd_vel']),
                ('/odom', [robot_name, '/odom'])
            ]
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            parameters=[{'robot_description': 'robot.urdf'}]
        ),
        
        # Control node with conditional parameters
        Node(
            package='my_robot',
            executable='control_node',
            name='controller',
            namespace=robot_name,
            parameters=[
                config_file,
                {
                    'max_speed': 2.0 if use_simulation else 1.0,
                    'safety_distance': 0.5
                }
            ]
        )
    ])
```

## Understanding Parameters

Parameters are configuration values that can be set for nodes without modifying code. They allow runtime customization of node behavior.

### Parameter Types

```python
# Basic parameter types
string_param = 'hello_world'
int_param = 42
float_param = 3.14159
bool_param = True
array_param = [1, 2, 3, 4, 5]
```

### Parameter Declaration in Nodes

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')
        
        # Declare parameters with descriptions and constraints
        self.declare_parameter(
            'robot_name',
            'default_robot',
            ParameterDescriptor(description='Name of the robot')
        )
        
        self.declare_parameter(
            'max_speed',
            1.0,
            ParameterDescriptor(
                description='Maximum robot speed in m/s',
                floating_point_range=[
                    {'from_value': 0.1, 'to_value': 5.0}
                ]
            )
        )
        
        self.declare_parameter(
            'sensor_topics',
            ['/camera/image', '/lidar/scan'],
            ParameterDescriptor(description='List of sensor topics')
        )
        
        self.declare_parameter(
            'enable_safety',
            True,
            ParameterDescriptor(description='Enable safety features')
        )
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.sensor_topics = self.get_parameter('sensor_topics').value
        self.safety_enabled = self.get_parameter('enable_safety').value
        
        # Set up parameter callback for runtime changes
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info(f'Node configured: {self.robot_name}')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')
    
    def parameter_callback(self, params):
        """Handle parameter changes at runtime"""
        for param in params:
            if param.name == 'max_speed':
                if 0.1 <= param.value <= 5.0:
                    self.max_speed = param.value
                    self.get_logger().info(f'Speed updated to {param.value} m/s')
                else:
                    self.get_logger().error('Invalid speed value')
                    return SetParametersResult(successful=False)
            
            elif param.name == 'enable_safety':
                self.safety_enabled = param.value
                self.get_logger().info(f'Safety {"enabled" if param.value else "disabled"}')
        
        return SetParametersResult(successful=True)
```

## Parameter Configuration Files

### YAML Parameter File

**File: `config/robot_params.yaml`**
```yaml
# Robot configuration
robot_name: "explorer_robot"
max_speed: 2.5
enable_safety: true

# Sensor configuration
camera:
  frame_rate: 30
  resolution: [1920, 1080]
  auto_exposure: true

lidar:
  range_min: 0.1
  range_max: 10.0
  angle_increment: 0.25
  scan_frequency: 10.0

# Navigation parameters
navigation:
  goal_tolerance: 0.1
  path_resolution: 0.05
  max_planning_time: 5.0
  
# Safety parameters
safety:
  emergency_stop_distance: 0.3
  warning_distance: 0.8
  max_acceleration: 2.0

# Network configuration
network:
  robot_ip: "192.168.1.100"
  base_station_ip: "192.168.1.1"
  port: 8080
```

### Using Parameter Files in Launch

```python
# In launch file
config_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')

Node(
    package='my_robot',
    executable='navigation_node',
    parameters=[config_file]  # Load all parameters from file
)

# Or load specific namespace
Node(
    package='my_robot',
    executable='camera_node',
    parameters=[{
        'camera': config_file  # Load only camera parameters
    }]
)
```

## Advanced Launch Features

### Conditional Node Launch

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

# Launch node only if condition is true
Node(
    package='my_robot',
    executable='simulation_node',
    condition=IfCondition(LaunchConfiguration('use_simulation'))
)

# Launch node only if condition is false
Node(
    package='my_robot',
    executable='hardware_driver',
    condition=UnlessCondition(LaunchConfiguration('use_simulation'))
)
```

### Including Other Launch Files

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Include another launch file
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('nav2_bringup'),
        '/launch/navigation_launch.py'
    ]),
    launch_arguments={
        'use_sim_time': 'true',
        'params_file': config_file
    }.items()
)
```

### Environment Variables

```python
from launch.actions import SetEnvironmentVariable

# Set environment variable
SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)
```

### Delayed Node Launch

```python
from launch.actions import TimerAction

# Launch node after 5 seconds
TimerAction(
    period=5.0,
    actions=[
        Node(
            package='my_robot',
            executable='delayed_node'
        )
    ]
)
```

## Parameter Management at Runtime

### Command Line Tools

```bash
# List all parameters
ros2 param list

# Get parameter value
ros2 param get /robot1/navigation max_speed

# Set parameter value
ros2 param set /robot1/navigation max_speed 1.5

# Dump all parameters to file
ros2 param dump /robot1/navigation --output-dir ./config/

# Load parameters from file
ros2 param load /robot1/navigation ./config/navigation.yaml
```

### Programmatic Parameter Access

```python
# In another node, access parameters from different node
class ParameterClient(Node):
    def __init__(self):
        super().__init__('param_client')
        
        # Create parameter client for another node
        self.param_client = self.create_client(
            GetParameters,
            '/robot1/navigation/get_parameters'
        )
    
    def get_remote_parameter(self, param_name):
        """Get parameter from another node"""
        request = GetParameters.Request()
        request.names = [param_name]
        
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            return future.result().values[0]
        return None
```

## Launch File Organization

### Project Structure
```
my_robot_package/
├── launch/
│   ├── robot_bringup.launch.py      # Main robot launch
│   ├── simulation.launch.py         # Simulation-specific
│   ├── navigation.launch.py         # Navigation stack
│   └── sensors.launch.py            # Sensor nodes
├── config/
│   ├── robot_params.yaml           # Main parameters
│   ├── simulation_params.yaml      # Simulation overrides
│   └── navigation_params.yaml      # Navigation config
└── src/
    └── my_robot_package/
        ├── nodes/                   # Node implementations
        └── launch/                  # Launch utilities
```

### Modular Launch Files

**Main Launch File: `robot_bringup.launch.py`**
```python
def generate_launch_description():
    return LaunchDescription([
        # Include sensor launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('my_robot'),
                '/launch/sensors.launch.py'
            ])
        ),
        
        # Include navigation launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('my_robot'),
                '/launch/navigation.launch.py'
            ])
        )
    ])
```

## Debugging Launch Files

### Launch File Testing

```bash
# Test launch file syntax
ros2 launch my_robot robot_bringup.launch.py --show-args

# Launch with specific arguments
ros2 launch my_robot robot_bringup.launch.py robot_name:=test_robot debug:=true

# Show what would be launched (dry run)
ros2 launch my_robot robot_bringup.launch.py --show-launch-tree
```

### Common Issues and Solutions

1. **Parameter not found**
   ```python
   # Wrong: Parameter not declared
   speed = self.get_parameter('max_speed').value  # Error!
   
   # Correct: Declare first
   self.declare_parameter('max_speed', 1.0)
   speed = self.get_parameter('max_speed').value
   ```

2. **File path errors**
   ```python
   # Wrong: Hardcoded path
   config_file = '/home/user/config/params.yaml'  # Won't work on other systems
   
   # Correct: Use package path
   config_file = os.path.join(
       get_package_share_directory('my_robot'),
       'config', 'params.yaml'
   )
   ```

3. **Namespace conflicts**
   ```python
   # Use namespaces to avoid conflicts
   Node(
       package='my_robot',
       executable='sensor_node',
       namespace='robot1'  # Prevents conflicts with robot2
   )
   ```

## Best Practices

### Launch Files
1. **Use descriptive names**: `robot_navigation.launch.py` not `launch1.py`
2. **Modularize**: Split complex launches into smaller, focused files
3. **Document arguments**: Provide clear descriptions for all launch arguments
4. **Use conditions**: Support different deployment scenarios
5. **Validate paths**: Check that config files exist

### Parameters
1. **Declare all parameters**: Don't use undeclared parameters
2. **Provide defaults**: Always have sensible default values
3. **Add descriptions**: Document what each parameter does
4. **Validate ranges**: Set min/max values where appropriate
5. **Group related parameters**: Use namespaces to organize

## Practical Exercise

Create a launch system for a mobile robot with:

1. **Sensor nodes**: Camera and LIDAR
2. **Navigation node**: Path planning and control
3. **Safety node**: Emergency stop and collision avoidance
4. **Parameter files**: Separate configs for simulation and real robot
5. **Launch arguments**: Robot name, simulation mode, debug level

**Requirements:**
- Support both simulation and real hardware
- Allow runtime parameter changes
- Include proper error handling
- Use modular launch file structure

## Key Takeaways

- Launch files automate starting multiple nodes with proper configuration
- Parameters allow runtime customization without code changes
- YAML files provide organized parameter storage
- Conditional launching supports different deployment scenarios
- Proper parameter declaration and validation prevent runtime errors
- Modular launch files improve maintainability and reusability
- Command-line tools enable runtime parameter management

---

**Next:** [Lecture 5: Building ROS 2 Packages](./lecture-5.md)