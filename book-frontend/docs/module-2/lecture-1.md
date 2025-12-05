---
sidebar_position: 1
---

# Lecture 1: Introduction to ROS 2

## What is ROS 2?

**ROS 2** (Robot Operating System 2) is not actually an operating system like Windows or Linux. Instead, it's a **middleware framework** that provides tools, libraries, and conventions for building robot applications. Think of it as the "nervous system" that connects all parts of a robot.

## Why Do We Need ROS 2?

### The Robot Complexity Problem

Modern robots are incredibly complex systems with many components:
- Multiple sensors (cameras, LIDAR, IMU)
- Various actuators (motors, servos, grippers)
- AI processing units
- Communication systems
- Safety systems

**Without ROS 2**: Each component would need custom integration
**With ROS 2**: Standardized communication and tools

### Real-World Analogy: The Human Nervous System

```
Human Body:
Brain ←→ Nervous System ←→ Eyes, Ears, Muscles, Organs

Robot System:
AI Computer ←→ ROS 2 ←→ Sensors, Motors, Actuators
```

Just like your nervous system coordinates between your brain and body parts, ROS 2 coordinates between the robot's computer and its physical components.

## ROS 1 vs ROS 2: Why the Upgrade?

### ROS 1 Limitations
- **Single point of failure**: If master node crashes, entire system fails
- **No real-time support**: Unpredictable timing
- **Limited security**: Not suitable for commercial applications
- **Python 2 dependency**: Outdated technology

### ROS 2 Improvements
- **Distributed system**: No single point of failure
- **Real-time capable**: Deterministic timing for critical applications
- **Built-in security**: Encryption and authentication
- **Modern languages**: Python 3, C++17 support
- **Cross-platform**: Works on Linux, Windows, macOS

## Core Concepts of ROS 2

### 1. Nodes
**Definition**: Independent processes that perform specific tasks

**Examples:**
- Camera node: Captures and publishes images
- Motor controller node: Receives commands and moves motors
- AI planning node: Makes decisions about robot actions

**Analogy**: Like apps on your smartphone - each has a specific function

```python
# Simple ROS 2 node structure
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        self.get_logger().info('Robot node started!')

def main():
    rclpy.init()
    node = MyRobotNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 2. Topics
**Definition**: Named channels for data communication between nodes

**Characteristics:**
- **Publisher**: Node that sends data
- **Subscriber**: Node that receives data
- **Many-to-many**: Multiple publishers and subscribers allowed

**Example**: Camera publishes images to `/camera/image` topic
```python
# Publisher example
self.image_publisher = self.create_publisher(Image, '/camera/image', 10)

# Subscriber example
self.image_subscriber = self.create_subscription(
    Image, '/camera/image', self.image_callback, 10)
```

### 3. Services
**Definition**: Request-response communication for immediate actions

**Use cases:**
- Turn on/off a component
- Request current robot status
- Trigger a specific action

**Example**: Service to move robot to a specific position
```python
# Service server (provides the service)
self.move_service = self.create_service(
    MoveRobot, 'move_to_position', self.move_callback)

# Service client (uses the service)
self.move_client = self.create_client(MoveRobot, 'move_to_position')
```

### 4. Actions
**Definition**: Long-running tasks with feedback and cancellation

**Use cases:**
- Navigate to a destination (takes time, can be cancelled)
- Pick up an object (multiple steps, progress updates)
- Charge battery (long duration, status updates)

**Components:**
- **Goal**: What you want to achieve
- **Feedback**: Progress updates
- **Result**: Final outcome

## ROS 2 Communication Patterns

### 1. Publish-Subscribe (Topics)
```
Sensor Node → [Topic: /sensor_data] → Processing Node
                                   → Logging Node
                                   → Display Node
```
**Best for**: Continuous data streams (sensor readings, status updates)

### 2. Request-Response (Services)
```
Client Node → [Service Request] → Server Node
Client Node ← [Service Response] ← Server Node
```
**Best for**: Immediate actions (turn on light, get current position)

### 3. Goal-Oriented (Actions)
```
Client → [Goal] → Action Server
Client ← [Feedback] ← Action Server (ongoing)
Client ← [Result] ← Action Server (final)
```
**Best for**: Long-running tasks (navigation, manipulation)

## ROS 2 Workspace Structure

### Typical ROS 2 Project Layout
```
my_robot_workspace/
├── src/                    # Source code
│   ├── my_robot_pkg/      # Your robot package
│   │   ├── my_robot_pkg/  # Python modules
│   │   ├── launch/        # Launch files
│   │   ├── config/        # Configuration files
│   │   └── package.xml    # Package description
│   └── another_pkg/
├── build/                 # Compiled code (auto-generated)
├── install/               # Installed packages (auto-generated)
└── log/                   # Build and runtime logs
```

### Package Structure
```
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py
│   ├── robot_controller.py    # Main robot logic
│   └── sensor_processor.py    # Sensor data processing
├── launch/
│   └── robot_launch.py        # Launch configuration
├── config/
│   └── robot_params.yaml      # Parameters
├── package.xml                # Package metadata
└── setup.py                   # Python package setup
```

## Key ROS 2 Tools

### 1. Command Line Interface (CLI)
```bash
# List running nodes
ros2 node list

# See available topics
ros2 topic list

# Monitor topic data
ros2 topic echo /camera/image

# Call a service
ros2 service call /move_robot geometry_msgs/srv/Twist

# Launch a robot system
ros2 launch my_robot_pkg robot_launch.py
```

### 2. RViz2 - Visualization Tool
**Purpose**: 3D visualization of robot data
**Features**:
- Display sensor data (LIDAR, camera, point clouds)
- Show robot model and joint states
- Visualize navigation paths and goals
- Debug robot behavior in real-time

### 3. Gazebo - Simulation Environment
**Purpose**: Physics-based robot simulation
**Features**:
- Realistic physics simulation
- Sensor simulation (cameras, LIDAR, IMU)
- Multiple robot support
- Custom world creation

## ROS 2 in Action: Simple Example

### Scenario: Temperature Monitoring Robot

**Components needed:**
1. **Temperature sensor node**: Reads temperature
2. **Alert node**: Monitors temperature and sends alerts
3. **Display node**: Shows current temperature

**Implementation:**
```python
# Temperature sensor node (publisher)
class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
    
    def publish_temperature(self):
        temp = self.read_sensor()  # Read actual sensor
        msg = Float32()
        msg.data = temp
        self.publisher.publish(msg)

# Alert node (subscriber)
class TemperatureAlert(Node):
    def __init__(self):
        super().__init__('temperature_alert')
        self.subscription = self.create_subscription(
            Float32, 'temperature', self.temperature_callback, 10)
    
    def temperature_callback(self, msg):
        if msg.data > 30.0:  # Alert if temperature > 30°C
            self.get_logger().warn(f'High temperature: {msg.data}°C')
```

## Benefits of Using ROS 2

### 1. Modularity
- Each component is independent
- Easy to test individual parts
- Reusable across different robots

### 2. Scalability
- Add new sensors without changing existing code
- Distribute processing across multiple computers
- Support for robot swarms

### 3. Community and Ecosystem
- Thousands of existing packages
- Active community support
- Industry standard for robotics

### 4. Language Flexibility
- Write nodes in Python, C++, or other supported languages
- Mix languages in the same system
- Use the best language for each task

## Common ROS 2 Packages

### Navigation Stack (Nav2)
- **Purpose**: Autonomous navigation
- **Features**: Path planning, obstacle avoidance, localization

### MoveIt 2
- **Purpose**: Motion planning for robot arms
- **Features**: Collision detection, trajectory planning, kinematics

### Robot State Publisher
- **Purpose**: Publishes robot joint states and transforms
- **Use**: Essential for robot visualization and control

### Joint State Publisher
- **Purpose**: Publishes joint positions for robot visualization
- **Use**: Testing and simulation

## Getting Started Checklist

### Installation Requirements
- **Operating System**: Ubuntu 22.04 (recommended) or other supported OS
- **ROS 2 Distribution**: Humble Hawksbill (LTS) or Iron Irwini
- **Python**: 3.8 or newer
- **Build Tools**: colcon, rosdep

### First Steps
1. **Install ROS 2**: Follow official installation guide
2. **Set up workspace**: Create and build your first workspace
3. **Run examples**: Try built-in demo nodes
4. **Create simple node**: Write your first publisher/subscriber
5. **Explore tools**: Use CLI tools to inspect running system

## Real-World Applications

### Industrial Robots
- **Use case**: Factory automation, assembly lines
- **ROS 2 role**: Coordinate multiple robots, integrate with factory systems

### Autonomous Vehicles
- **Use case**: Self-driving cars, delivery robots
- **ROS 2 role**: Process sensor data, plan paths, control actuators

### Service Robots
- **Use case**: Cleaning robots, security robots, assistive robots
- **ROS 2 role**: Navigate environments, interact with humans

### Research Robots
- **Use case**: University research, algorithm development
- **ROS 2 role**: Rapid prototyping, data collection, experimentation

## Key Takeaways

- ROS 2 is a middleware framework that connects robot components
- It provides standardized communication through nodes, topics, services, and actions
- ROS 2 improves on ROS 1 with better reliability, security, and real-time support
- The modular design makes robot systems easier to develop, test, and maintain
- A rich ecosystem of packages accelerates robot development
- ROS 2 is the industry standard for modern robotics applications

---

**Next:** [Lecture 2: Nodes, Topics, and Communication](./lecture-2.md)