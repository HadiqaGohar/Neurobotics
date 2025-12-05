---
sidebar_position: 2
---

# Lecture 2: Nodes, Topics, and Communication

## Deep Dive into ROS 2 Nodes

### What Makes a Good Node?

A well-designed ROS 2 node follows the **Single Responsibility Principle** - each node should do one thing and do it well.

**Good Examples:**
- Camera driver node: Only handles camera communication
- Image processor node: Only processes images
- Motor controller node: Only controls motors

**Bad Example:**
- Mega node: Handles camera, processes images, controls motors, plans paths
  - Hard to debug
  - Difficult to reuse
  - Single point of failure

### Node Lifecycle

ROS 2 nodes have a defined lifecycle with different states:

```
Unconfigured → Inactive → Active → Inactive → Finalized
     ↑            ↑         ↑         ↑         ↑
  configure   activate   deactivate  cleanup  shutdown
```

**States Explained:**
- **Unconfigured**: Node exists but not ready
- **Inactive**: Configured but not processing data
- **Active**: Fully operational
- **Finalized**: Shutting down

### Creating Your First Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldNode(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('hello_world_node')
        
        # Create a publisher
        self.publisher = self.create_publisher(String, 'hello_topic', 10)
        
        # Create a timer to publish messages regularly
        self.timer = self.create_timer(2.0, self.publish_message)
        
        # Initialize message counter
        self.counter = 0
        
        # Log that the node started
        self.get_logger().info('Hello World Node has started!')
    
    def publish_message(self):
        # Create and populate message
        msg = String()
        msg.data = f'Hello World! Message #{self.counter}'
        
        # Publish the message
        self.publisher.publish(msg)
        
        # Log the published message
        self.get_logger().info(f'Published: {msg.data}')
        
        # Increment counter
        self.counter += 1

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create the node
    node = HelloWorldNode()
    
    # Keep the node running
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Clean shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Understanding Topics in Detail

### Topic Naming Conventions

**Good Topic Names:**
```
/robot/sensors/camera/image
/robot/actuators/wheels/velocity
/robot/status/battery_level
/environment/temperature
```

**Bad Topic Names:**
```
/data          # Too generic
/cam           # Unclear abbreviation
/robot_stuff   # Not descriptive
```

### Topic Types and Message Structure

#### Standard Message Types
```python
# Basic types
from std_msgs.msg import String, Int32, Float64, Bool

# Geometry types
from geometry_msgs.msg import Point, Pose, Twist, Vector3

# Sensor types
from sensor_msgs.msg import Image, LaserScan, Imu, JointState
```

#### Custom Message Example
Create a custom message for robot status:

**File: `my_robot_msgs/msg/RobotStatus.msg`**
```
# Robot identification
string robot_name
int32 robot_id

# Status information
float64 battery_percentage
float64 temperature
bool is_moving
bool emergency_stop

# Position information
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# Timestamp
builtin_interfaces/Time timestamp
```

**Using the custom message:**
```python
from my_robot_msgs.msg import RobotStatus
from geometry_msgs.msg import Point, Vector3

def create_status_message(self):
    msg = RobotStatus()
    msg.robot_name = "Explorer-1"
    msg.robot_id = 42
    msg.battery_percentage = 85.5
    msg.temperature = 23.7
    msg.is_moving = True
    msg.emergency_stop = False
    
    # Set position
    msg.position = Point()
    msg.position.x = 1.5
    msg.position.y = 2.3
    msg.position.z = 0.0
    
    # Set velocity
    msg.velocity = Vector3()
    msg.velocity.x = 0.5
    msg.velocity.y = 0.0
    msg.velocity.z = 0.0
    
    # Set timestamp
    msg.timestamp = self.get_clock().now().to_msg()
    
    return msg
```

## Publisher-Subscriber Pattern

### Quality of Service (QoS)

QoS settings control how messages are delivered:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Reliable communication (guaranteed delivery)
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Best effort (faster, may lose messages)
fast_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Create publisher with specific QoS
self.publisher = self.create_publisher(
    Image, 
    '/camera/image', 
    reliable_qos
)
```

### Complete Publisher Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from rclpy.qos import QoSProfile, ReliabilityPolicy
import random

class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        
        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # Create publisher
        self.publisher = self.create_publisher(
            Temperature,
            '/sensors/temperature',
            sensor_qos
        )
        
        # Publish at 10 Hz (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.publish_temperature)
        
        self.get_logger().info('Temperature sensor node started')
    
    def publish_temperature(self):
        # Simulate temperature reading (20-30°C with noise)
        temp_celsius = 25.0 + random.uniform(-5.0, 5.0)
        
        # Create message
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'temperature_sensor'
        msg.temperature = temp_celsius
        msg.variance = 0.1  # Sensor uncertainty
        
        # Publish
        self.publisher.publish(msg)
        
        # Log occasionally (every 50 messages = 5 seconds)
        if hasattr(self, 'msg_count'):
            self.msg_count += 1
        else:
            self.msg_count = 1
            
        if self.msg_count % 50 == 0:
            self.get_logger().info(f'Temperature: {temp_celsius:.1f}°C')
```

### Complete Subscriber Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from rclpy.qos import QoSProfile, ReliabilityPolicy

class TemperatureMonitorNode(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        
        # Match publisher's QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Temperature,
            '/sensors/temperature',
            self.temperature_callback,
            sensor_qos
        )
        
        # Temperature thresholds
        self.min_temp = 15.0  # °C
        self.max_temp = 35.0  # °C
        
        # Statistics
        self.temp_readings = []
        self.max_readings = 100  # Keep last 100 readings
        
        self.get_logger().info('Temperature monitor node started')
    
    def temperature_callback(self, msg):
        temp = msg.temperature
        
        # Store reading for statistics
        self.temp_readings.append(temp)
        if len(self.temp_readings) > self.max_readings:
            self.temp_readings.pop(0)  # Remove oldest
        
        # Check thresholds
        if temp < self.min_temp:
            self.get_logger().warn(f'LOW TEMPERATURE: {temp:.1f}°C')
        elif temp > self.max_temp:
            self.get_logger().warn(f'HIGH TEMPERATURE: {temp:.1f}°C')
        
        # Calculate and log statistics every 50 readings
        if len(self.temp_readings) % 50 == 0:
            avg_temp = sum(self.temp_readings) / len(self.temp_readings)
            min_temp = min(self.temp_readings)
            max_temp = max(self.temp_readings)
            
            self.get_logger().info(
                f'Stats - Avg: {avg_temp:.1f}°C, '
                f'Min: {min_temp:.1f}°C, '
                f'Max: {max_temp:.1f}°C'
            )
```

## Advanced Communication Patterns

### Multiple Publishers, Single Subscriber

```python
# Multiple temperature sensors publishing to same topic
# Monitor node subscribes once and receives all data

# Sensor 1 publishes to: /sensors/temperature
# Sensor 2 publishes to: /sensors/temperature  
# Sensor 3 publishes to: /sensors/temperature
# Monitor subscribes to: /sensors/temperature (receives all)
```

### Single Publisher, Multiple Subscribers

```python
# Camera publishes images once
# Multiple nodes process the same images

# Camera publishes to: /camera/image
# Face detector subscribes to: /camera/image
# Object detector subscribes to: /camera/image
# Image saver subscribes to: /camera/image
```

### Topic Remapping

Change topic names without modifying code:

```bash
# Run node with different topic name
ros2 run my_package temperature_sensor --ros-args -r /sensors/temperature:=/robot1/sensors/temperature

# Or in launch file
Node(
    package='my_package',
    executable='temperature_sensor',
    remappings=[('/sensors/temperature', '/robot1/sensors/temperature')]
)
```

## Debugging Communication

### Command Line Tools

```bash
# List all active topics
ros2 topic list

# Show topic information
ros2 topic info /sensors/temperature

# Monitor topic data in real-time
ros2 topic echo /sensors/temperature

# Check message rate
ros2 topic hz /sensors/temperature

# Show topic type
ros2 topic type /sensors/temperature

# Publish test message
ros2 topic pub /sensors/temperature sensor_msgs/msg/Temperature "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'test'
temperature: 25.5
variance: 0.1"
```

### Programmatic Debugging

```python
class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')
        
        # Subscribe to multiple topics for debugging
        self.create_subscription(Temperature, '/sensors/temperature', 
                               self.temp_callback, 10)
        self.create_subscription(Image, '/camera/image', 
                               self.image_callback, 10)
        
        # Create timer to check connection status
        self.create_timer(5.0, self.check_connections)
        
        self.temp_count = 0
        self.image_count = 0
    
    def temp_callback(self, msg):
        self.temp_count += 1
    
    def image_callback(self, msg):
        self.image_count += 1
    
    def check_connections(self):
        self.get_logger().info(
            f'Received - Temperature: {self.temp_count}, '
            f'Images: {self.image_count}'
        )
        # Reset counters
        self.temp_count = 0
        self.image_count = 0
```

## Best Practices for Topics

### 1. Topic Design
- **Use descriptive names**: `/robot/sensors/lidar/scan` not `/scan`
- **Follow hierarchy**: `/robot_name/component/sensor/data_type`
- **Be consistent**: Use same naming pattern across project

### 2. Message Design
- **Keep messages focused**: One message type per logical data unit
- **Include timestamps**: Always add header with timestamp
- **Add metadata**: Frame IDs, sequence numbers, quality indicators

### 3. Performance Optimization
- **Choose appropriate QoS**: Reliable for critical data, best-effort for high-frequency
- **Limit message size**: Large messages slow down communication
- **Use appropriate rates**: Don't publish faster than consumers can process

### 4. Error Handling
```python
def safe_callback(self, msg):
    try:
        # Process message
        self.process_data(msg)
    except Exception as e:
        self.get_logger().error(f'Error processing message: {e}')
        # Continue running, don't crash
```

## Practical Exercise: Robot Sensor Network

**Goal**: Create a simple sensor network with multiple sensors and a central monitor.

**Components to implement:**
1. **Temperature sensor node**: Publishes temperature data
2. **Humidity sensor node**: Publishes humidity data  
3. **Battery monitor node**: Publishes battery status
4. **Central monitor node**: Subscribes to all sensors and displays status

**Your task:**
- Create the four nodes
- Use appropriate message types
- Implement proper error handling
- Add debugging and logging
- Test with ROS 2 CLI tools

## Key Takeaways

- Nodes are independent processes that communicate via topics
- Topics use publish-subscribe pattern for data streaming
- QoS settings control message delivery guarantees
- Good naming conventions make systems easier to understand
- Multiple publishers and subscribers can share the same topic
- ROS 2 CLI tools are essential for debugging communication
- Proper error handling keeps systems robust

---

**Next:** [Lecture 3: Services and Actions](./lecture-3.md)