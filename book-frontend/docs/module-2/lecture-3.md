---
sidebar_position: 3
---

# Lecture 3: Services and Actions

## Understanding Services

Services provide **request-response** communication in ROS 2. Unlike topics that stream continuous data, services are used for:
- Immediate actions that need confirmation
- Getting current status or configuration
- Triggering specific behaviors
- One-time data requests

### Service vs Topic Comparison

| Aspect | Topics | Services |
|--------|--------|----------|
| **Communication** | One-way streaming | Two-way request-response |
| **Timing** | Continuous | On-demand |
| **Subscribers** | Multiple | One server, multiple clients |
| **Use Case** | Sensor data, status updates | Commands, queries |
| **Example** | Camera images | "Turn on LED" |

## Service Structure

Every ROS 2 service has three parts:

### 1. Service Definition (.srv file)
```
# Request part (what client sends)
string command
float64 value
---
# Response part (what server returns)
bool success
string message
float64 result
```

### 2. Service Server
The node that **provides** the service (does the work)

### 3. Service Client  
The node that **uses** the service (makes requests)

## Creating Custom Services

### Step 1: Define Service Type

**File: `my_robot_msgs/srv/SetLED.srv`**
```
# Request: Which LED and what state
string led_name
bool turn_on
uint8 brightness  # 0-255
---
# Response: Success status and message
bool success
string message
```

**File: `my_robot_msgs/srv/GetRobotStatus.srv`**
```
# Request: Empty (just asking for status)
---
# Response: Current robot status
float64 battery_percentage
geometry_msgs/Point position
bool is_moving
string current_task
builtin_interfaces/Time last_update
```

### Step 2: Build the Service Messages
```bash
# In your workspace
colcon build --packages-select my_robot_msgs
source install/setup.bash
```

## Service Server Implementation

### Simple LED Control Server

```python
import rclpy
from rclpy.node import Node
from my_robot_msgs.srv import SetLED

class LEDControllerNode(Node):
    def __init__(self):
        super().__init__('led_controller')
        
        # Create service server
        self.service = self.create_service(
            SetLED,                    # Service type
            'set_led',                 # Service name
            self.set_led_callback      # Callback function
        )
        
        # Simulate LED states (in real robot, this would control hardware)
        self.led_states = {
            'status_led': {'on': False, 'brightness': 0},
            'warning_led': {'on': False, 'brightness': 0},
            'power_led': {'on': True, 'brightness': 255}  # Always on
        }
        
        self.get_logger().info('LED Controller service ready')
    
    def set_led_callback(self, request, response):
        """Handle LED control requests"""
        led_name = request.led_name
        turn_on = request.turn_on
        brightness = request.brightness
        
        # Validate LED name
        if led_name not in self.led_states:
            response.success = False
            response.message = f"Unknown LED: {led_name}"
            self.get_logger().warn(f"Request for unknown LED: {led_name}")
            return response
        
        # Validate brightness
        if brightness < 0 or brightness > 255:
            response.success = False
            response.message = "Brightness must be between 0 and 255"
            return response
        
        # Special case: power LED cannot be turned off
        if led_name == 'power_led' and not turn_on:
            response.success = False
            response.message = "Power LED cannot be turned off"
            return response
        
        # Update LED state
        try:
            self.led_states[led_name]['on'] = turn_on
            if turn_on:
                self.led_states[led_name]['brightness'] = brightness
            else:
                self.led_states[led_name]['brightness'] = 0
            
            # In real robot, you would control actual hardware here
            # self.hardware_controller.set_led(led_name, turn_on, brightness)
            
            response.success = True
            state = "ON" if turn_on else "OFF"
            response.message = f"LED '{led_name}' set to {state} (brightness: {brightness})"
            
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Error controlling LED: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
```

### Robot Status Server

```python
import rclpy
from rclpy.node import Node
from my_robot_msgs.srv import GetRobotStatus
from geometry_msgs.msg import Point
import time

class RobotStatusNode(Node):
    def __init__(self):
        super().__init__('robot_status')
        
        # Create service
        self.service = self.create_service(
            GetRobotStatus,
            'get_robot_status',
            self.get_status_callback
        )
        
        # Simulate robot state
        self.battery_level = 85.5
        self.position = Point(x=1.2, y=3.4, z=0.0)
        self.is_moving = False
        self.current_task = "idle"
        
        # Update robot state periodically
        self.create_timer(1.0, self.update_robot_state)
        
        self.get_logger().info('Robot Status service ready')
    
    def update_robot_state(self):
        """Simulate robot state changes"""
        # Slowly drain battery
        self.battery_level -= 0.01
        if self.battery_level < 0:
            self.battery_level = 100.0  # Simulate recharge
        
        # Simulate movement
        import random
        if random.random() < 0.1:  # 10% chance to change movement
            self.is_moving = not self.is_moving
            if self.is_moving:
                self.current_task = "navigating"
                # Simulate position change
                self.position.x += random.uniform(-0.1, 0.1)
                self.position.y += random.uniform(-0.1, 0.1)
            else:
                self.current_task = "idle"
    
    def get_status_callback(self, request, response):
        """Return current robot status"""
        response.battery_percentage = self.battery_level
        response.position = self.position
        response.is_moving = self.is_moving
        response.current_task = self.current_task
        response.last_update = self.get_clock().now().to_msg()
        
        self.get_logger().info(f'Status requested - Battery: {self.battery_level:.1f}%')
        
        return response
```

## Service Client Implementation

### LED Control Client

```python
import rclpy
from rclpy.node import Node
from my_robot_msgs.srv import SetLED
import sys

class LEDClientNode(Node):
    def __init__(self):
        super().__init__('led_client')
        
        # Create service client
        self.client = self.create_client(SetLED, 'set_led')
        
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LED service not available, waiting...')
    
    def send_led_request(self, led_name, turn_on, brightness=255):
        """Send LED control request"""
        # Create request
        request = SetLED.Request()
        request.led_name = led_name
        request.turn_on = turn_on
        request.brightness = brightness
        
        # Send request asynchronously
        future = self.client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.get_logger().error(f'Failed: {response.message}')
            return response
        else:
            self.get_logger().error('Service call failed')
            return None

def main():
    rclpy.init()
    
    # Parse command line arguments
    if len(sys.argv) != 4:
        print("Usage: ros2 run my_package led_client <led_name> <on/off> <brightness>")
        return
    
    led_name = sys.argv[1]
    turn_on = sys.argv[2].lower() == 'on'
    brightness = int(sys.argv[3])
    
    # Create client and send request
    client = LEDClientNode()
    client.send_led_request(led_name, turn_on, brightness)
    
    client.destroy_node()
    rclpy.shutdown()
```

### Status Monitor Client

```python
import rclpy
from rclpy.node import Node
from my_robot_msgs.srv import GetRobotStatus

class StatusMonitorNode(Node):
    def __init__(self):
        super().__init__('status_monitor')
        
        # Create service client
        self.client = self.create_client(GetRobotStatus, 'get_robot_status')
        
        # Request status every 5 seconds
        self.timer = self.create_timer(5.0, self.request_status)
        
        self.get_logger().info('Status Monitor started')
    
    def request_status(self):
        """Request robot status periodically"""
        if not self.client.service_is_ready():
            self.get_logger().warn('Status service not available')
            return
        
        # Create empty request
        request = GetRobotStatus.Request()
        
        # Send request
        future = self.client.call_async(request)
        future.add_done_callback(self.status_response_callback)
    
    def status_response_callback(self, future):
        """Handle status response"""
        try:
            response = future.result()
            
            # Display status
            self.get_logger().info(
                f'Robot Status:\n'
                f'  Battery: {response.battery_percentage:.1f}%\n'
                f'  Position: ({response.position.x:.2f}, {response.position.y:.2f})\n'
                f'  Moving: {response.is_moving}\n'
                f'  Task: {response.current_task}'
            )
            
            # Check for low battery
            if response.battery_percentage < 20.0:
                self.get_logger().warn('LOW BATTERY WARNING!')
                
        except Exception as e:
            self.get_logger().error(f'Status request failed: {e}')
```

## Understanding Actions

Actions are for **long-running tasks** that need:
- **Progress feedback** during execution
- **Ability to cancel** before completion
- **Final result** when finished

### Action vs Service Comparison

| Aspect | Services | Actions |
|--------|----------|---------|
| **Duration** | Immediate | Long-running |
| **Feedback** | None | Progress updates |
| **Cancellation** | Not possible | Can be cancelled |
| **Use Case** | Get status, simple commands | Navigation, manipulation |
| **Example** | "Turn on LED" | "Navigate to kitchen" |

## Action Structure

Actions have three message types:

### 1. Goal
What you want to achieve
```
# NavigateToPosition.action
geometry_msgs/Point target_position
float64 max_speed
---
# Result (when finished)
bool success
float64 final_distance_error
builtin_interfaces/Duration total_time
---
# Feedback (during execution)
geometry_msgs/Point current_position
float64 distance_remaining
float64 estimated_time_remaining
```

## Action Server Implementation

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from my_robot_msgs.action import NavigateToPosition
from geometry_msgs.msg import Point
import time
import math

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        
        # Create action server
        self.action_server = ActionServer(
            self,
            NavigateToPosition,
            'navigate_to_position',
            self.navigate_callback
        )
        
        # Robot state
        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.is_navigating = False
        
        self.get_logger().info('Navigation Action Server ready')
    
    def navigate_callback(self, goal_handle):
        """Handle navigation goal"""
        self.get_logger().info('Navigation goal received')
        
        # Get goal parameters
        target = goal_handle.request.target_position
        max_speed = goal_handle.request.max_speed
        
        # Validate goal
        if max_speed <= 0:
            goal_handle.abort()
            result = NavigateToPosition.Result()
            result.success = False
            return result
        
        # Accept the goal
        goal_handle.execute()
        
        # Simulate navigation
        return self.execute_navigation(goal_handle, target, max_speed)
    
    def execute_navigation(self, goal_handle, target, max_speed):
        """Execute the navigation task"""
        feedback = NavigateToPosition.Feedback()
        result = NavigateToPosition.Result()
        
        start_time = time.time()
        self.is_navigating = True
        
        # Calculate initial distance
        def distance_to_target():
            dx = target.x - self.current_position.x
            dy = target.y - self.current_position.y
            return math.sqrt(dx*dx + dy*dy)
        
        initial_distance = distance_to_target()
        
        # Navigation loop
        while distance_to_target() > 0.1:  # 10cm tolerance
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                self.is_navigating = False
                self.get_logger().info('Navigation cancelled')
                return result
            
            # Simulate movement toward target
            current_distance = distance_to_target()
            
            # Calculate direction
            dx = target.x - self.current_position.x
            dy = target.y - self.current_position.y
            
            # Normalize direction
            if current_distance > 0:
                dx /= current_distance
                dy /= current_distance
            
            # Move at max_speed (simulate with time step)
            time_step = 0.1  # 100ms
            move_distance = min(max_speed * time_step, current_distance)
            
            self.current_position.x += dx * move_distance
            self.current_position.y += dy * move_distance
            
            # Prepare feedback
            feedback.current_position = self.current_position
            feedback.distance_remaining = current_distance
            
            # Estimate remaining time
            if max_speed > 0:
                feedback.estimated_time_remaining = current_distance / max_speed
            
            # Publish feedback
            goal_handle.publish_feedback(feedback)
            
            # Log progress occasionally
            progress = (initial_distance - current_distance) / initial_distance * 100
            if int(progress) % 20 == 0:  # Every 20%
                self.get_logger().info(f'Navigation progress: {progress:.0f}%')
            
            # Sleep to simulate real movement
            time.sleep(time_step)
        
        # Navigation completed successfully
        self.is_navigating = False
        goal_handle.succeed()
        
        # Prepare result
        result.success = True
        result.final_distance_error = distance_to_target()
        result.total_time.sec = int(time.time() - start_time)
        result.total_time.nanosec = 0
        
        self.get_logger().info('Navigation completed successfully!')
        return result
```

## Action Client Implementation

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_msgs.action import NavigateToPosition
from geometry_msgs.msg import Point

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        
        # Create action client
        self.action_client = ActionClient(
            self,
            NavigateToPosition,
            'navigate_to_position'
        )
        
        self.get_logger().info('Navigation Client ready')
    
    def send_goal(self, x, y, speed=1.0):
        """Send navigation goal"""
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return
        
        # Create goal
        goal = NavigateToPosition.Goal()
        goal.target_position = Point(x=x, y=y, z=0.0)
        goal.max_speed = speed
        
        self.get_logger().info(f'Sending goal: ({x}, {y}) at {speed} m/s')
        
        # Send goal
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle progress feedback"""
        feedback = feedback_msg.feedback
        pos = feedback.current_position
        remaining = feedback.distance_remaining
        eta = feedback.estimated_time_remaining
        
        self.get_logger().info(
            f'Position: ({pos.x:.2f}, {pos.y:.2f}), '
            f'Remaining: {remaining:.2f}m, '
            f'ETA: {eta:.1f}s'
        )
    
    def result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        
        if result.success:
            self.get_logger().info(
                f'Navigation succeeded! '
                f'Error: {result.final_distance_error:.3f}m, '
                f'Time: {result.total_time.sec}s'
            )
        else:
            self.get_logger().error('Navigation failed')

def main():
    rclpy.init()
    
    client = NavigationClient()
    
    # Send navigation goal
    client.send_goal(5.0, 3.0, 2.0)  # Go to (5,3) at 2 m/s
    
    # Keep running to receive feedback and result
    rclpy.spin(client)
    
    client.destroy_node()
    rclpy.shutdown()
```

## Command Line Tools

### Services
```bash
# List available services
ros2 service list

# Get service type
ros2 service type /set_led

# Call service
ros2 service call /set_led my_robot_msgs/srv/SetLED "
led_name: 'status_led'
turn_on: true
brightness: 128"

# Find services of specific type
ros2 service find my_robot_msgs/srv/SetLED
```

### Actions
```bash
# List available actions
ros2 action list

# Get action info
ros2 action info /navigate_to_position

# Send action goal
ros2 action send_goal /navigate_to_position my_robot_msgs/action/NavigateToPosition "
target_position:
  x: 2.0
  y: 1.0
  z: 0.0
max_speed: 1.5"
```

## Best Practices

### Services
1. **Keep services fast**: Don't block for long operations
2. **Validate inputs**: Check all request parameters
3. **Provide meaningful responses**: Include success status and messages
4. **Handle errors gracefully**: Don't crash on invalid requests

### Actions
1. **Provide regular feedback**: Keep clients informed of progress
2. **Support cancellation**: Always check for cancel requests
3. **Set reasonable timeouts**: Don't run forever
4. **Clean up on failure**: Reset state when actions fail

## Key Takeaways

- Services provide request-response communication for immediate actions
- Actions handle long-running tasks with feedback and cancellation
- Service servers do the work, clients make requests
- Actions have goals, feedback, and results
- Both services and actions can be tested with command-line tools
- Proper error handling and validation are essential
- Choose the right communication pattern for your use case

---

**Next:** [Lecture 4: Launch Files and Parameters](./lecture-4.md)