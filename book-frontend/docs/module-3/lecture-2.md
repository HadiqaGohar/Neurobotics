---
sidebar_position: 2
---

# Lecture 2: Gazebo Fundamentals

## What is Gazebo?

Gazebo is a powerful 3D robot simulation environment that provides realistic physics, high-quality graphics, and extensive sensor simulation. It's the most popular simulator in the ROS ecosystem and is used by researchers, students, and companies worldwide.

## Installing Gazebo

### For Ubuntu 22.04 with ROS 2 Humble

```bash
# Install Gazebo Garden (recommended for ROS 2 Humble)
sudo apt update
sudo apt install gz-garden

# Install ROS 2 - Gazebo bridge
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim

# Verify installation
gz sim --version
```

### Alternative Installation Methods

```bash
# Install from source (for latest features)
git clone https://github.com/gazebosim/gz-sim
cd gz-sim
mkdir build && cd build
cmake .. && make -j4
sudo make install

# Using conda (cross-platform)
conda install -c conda-forge gazebo
```

## Gazebo Architecture

### Core Components

```
Gazebo Sim (gz-sim)
├── Physics Engine (gz-physics)
├── Rendering Engine (gz-rendering) 
├── GUI System (gz-gui)
├── Transport Layer (gz-transport)
├── Math Library (gz-math)
└── Plugin System (gz-plugin)
```

### Key Concepts

#### 1. Worlds
**Definition**: Complete simulation environments containing models, physics, and environmental settings

**Example World Structure:**
```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="robot_world">
    <!-- Physics settings -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

#### 2. Models
**Definition**: Individual objects in the simulation (robots, obstacles, furniture)

#### 3. Plugins
**Definition**: Code modules that extend Gazebo functionality

## Basic Gazebo Interface

### Starting Gazebo

```bash
# Start empty world
gz sim empty.sdf

# Start with specific world
gz sim worlds/shapes.sdf

# Start with GUI disabled (headless)
gz sim -s worlds/empty.sdf

# Start with specific physics engine
gz sim --physics-engine gz-physics-bullet-featherstone-plugin
```

### GUI Components

#### Main 3D View
- **Navigation**: Mouse controls for camera movement
- **Selection**: Click to select objects
- **Manipulation**: Move, rotate, scale objects

#### Entity Tree
- **Hierarchical view** of all simulation objects
- **Properties panel** for selected entities
- **Component inspector** for detailed settings

#### Plugin Panels
- **Topic Echo**: Monitor ROS 2 topics
- **Image Display**: View camera feeds
- **Plot**: Graph sensor data over time

## Creating Your First Simulation

### Step 1: Simple Robot World

```xml
<!-- File: simple_robot_world.sdf -->
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="simple_robot_world">
    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Simple box obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="box_link">
        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Step 2: Launch the World

```bash
# Save the above as simple_robot_world.sdf
gz sim simple_robot_world.sdf
```

## Working with Models

### Loading Existing Models

```bash
# List available models
gz model --list

# Download model from Gazebo Fuel
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X1 UGV"

# Spawn model in running simulation
gz service -s /world/simple_robot_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "X1_UGV"'
```

### Creating Custom Models

#### Simple Wheeled Robot Model

```xml
<!-- File: simple_robot.sdf -->
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="simple_robot">
    <pose>0 0 0.1 0 0 0</pose>
    
    <!-- Main body -->
    <link name="base_link">
      <collision name="base_collision">
        <geometry>
          <box>
            <size>0.6 0.4 0.2</size>
          </box>
        </geometry>
      </collision>
      
      <visual name="base_visual">
        <geometry>
          <box>
            <size>0.6 0.4 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient>
          <diffuse>0 0 1 1</diffuse>
        </material>
      </visual>
      
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.4</ixx>
          <iyy>0.4</iyy>
          <izz>0.2</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Left wheel -->
    <link name="left_wheel">
      <pose>0 0.25 0 -1.5707 0 0</pose>
      <collision name="left_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      
      <visual name="left_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
      
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.005</ixx>
          <iyy>0.005</iyy>
          <izz>0.005</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Right wheel -->
    <link name="right_wheel">
      <pose>0 -0.25 0 -1.5707 0 0</pose>
      <collision name="right_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      
      <visual name="right_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
      
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.005</ixx>
          <iyy>0.005</iyy>
          <izz>0.005</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Joints -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
    
    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
    
    <!-- Differential drive plugin -->
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.5</wheel_separation>
      <wheel_radius>0.1</wheel_radius>
      <odom_publish_frequency>1</odom_publish_frequency>
      <topic>cmd_vel</topic>
    </plugin>
  </model>
</sdf>
```

## Gazebo Plugins

### System Plugins (Built-in)

#### Physics System
```xml
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
</plugin>
```

#### Sensors System
```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

#### User Commands System
```xml
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands">
</plugin>
```

### Model Plugins

#### Differential Drive
```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.5</wheel_separation>
  <wheel_radius>0.1</wheel_radius>
  <odom_publish_frequency>50</odom_publish_frequency>
  <max_linear_acceleration>1</max_linear_acceleration>
  <min_linear_acceleration>-1</min_linear_acceleration>
  <max_angular_acceleration>2</max_angular_acceleration>
  <min_angular_acceleration>-2</min_angular_acceleration>
  <max_linear_velocity>0.5</max_linear_velocity>
  <min_linear_velocity>-0.5</min_linear_velocity>
  <max_angular_velocity>1</max_angular_velocity>
  <min_angular_velocity>-1</min_angular_velocity>
</plugin>
```

#### Joint Controller
```xml
<plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
  <joint_name>arm_joint_1</joint_name>
  <topic>arm_joint_1_cmd</topic>
  <p_gain>1000</p_gain>
  <i_gain>100</i_gain>
  <d_gain>10</d_gain>
</plugin>
```

## Sensor Simulation

### Camera Sensor

```xml
<sensor name="camera" type="camera">
  <pose>0.3 0 0.3 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <topic>camera</topic>
</sensor>
```

### LIDAR Sensor

```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.4 0 0 0</pose>
  <lidar>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-1.396263</min_angle>
        <max_angle>1.396263</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>0.01</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.08</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </lidar>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
  <topic>lidar</topic>
</sensor>
```

### IMU Sensor

```xml
<sensor name="imu" type="imu">
  <pose>0 0 0 0 0 0</pose>
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <visualize>true</visualize>
  <topic>imu</topic>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## ROS 2 Integration

### Gazebo-ROS Bridge

```bash
# Install bridge packages
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim

# Bridge specific topics
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

# Bridge multiple topics with config file
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config.yaml
```

### Bridge Configuration File

```yaml
# bridge_config.yaml
- ros_topic_name: "cmd_vel"
  gz_topic_name: "cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "odom"
  gz_topic_name: "odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "scan"
  gz_topic_name: "lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

### Launch File Integration

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot_gazebo')
    
    # World file path
    world_file = os.path.join(pkg_dir, 'worlds', 'simple_robot_world.sdf')
    
    return LaunchDescription([
        # Launch Gazebo with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ros_gz_sim'),
                '/launch/gz_sim.launch.py'
            ]),
            launch_arguments={
                'gz_args': world_file
            }.items()
        ),
        
        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'simple_robot'
            ]
        ),
        
        # Bridge topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ]
        )
    ])
```

## Command Line Tools

### Gazebo Commands

```bash
# List running simulations
gz sim --list

# Get simulation info
gz sim --info

# Pause/unpause simulation
gz service -s /world/simple_robot_world/control \
  --reqtype gz.msgs.WorldControl \
  --req 'pause: true'

# Reset simulation
gz service -s /world/simple_robot_world/control \
  --reqtype gz.msgs.WorldControl \
  --req 'reset: {all: true}'

# Step simulation
gz service -s /world/simple_robot_world/control \
  --reqtype gz.msgs.WorldControl \
  --req 'step: true'
```

### Topic Monitoring

```bash
# List Gazebo topics
gz topic -l

# Echo topic data
gz topic -e -t /cmd_vel

# Publish to topic
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.5}, angular: {z: 0.2}'

# Get topic info
gz topic -i -t /cmd_vel
```

## Performance Optimization

### Physics Settings

```xml
<!-- Optimize for speed -->
<physics name="fast" type="ignored">
  <max_step_size>0.01</max_step_size>  <!-- Larger steps = faster -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>100</real_time_update_rate>  <!-- Lower rate = faster -->
</physics>

<!-- Optimize for accuracy -->
<physics name="accurate" type="ignored">
  <max_step_size>0.001</max_step_size>  <!-- Smaller steps = more accurate -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

### Rendering Settings

```bash
# Run headless (no GUI)
gz sim -s world.sdf

# Reduce rendering quality
gz sim --render-engine ogre  # Instead of ogre2

# Limit sensor update rates
# In sensor definition:
<update_rate>10</update_rate>  <!-- Instead of 30 -->
```

## Troubleshooting Common Issues

### Installation Problems

```bash
# Check Gazebo installation
gz sim --version

# Check ROS 2 bridge
ros2 pkg list | grep ros_gz

# Reinstall if needed
sudo apt remove gz-garden
sudo apt install gz-garden
```

### Performance Issues

```bash
# Check system resources
htop  # Monitor CPU/RAM usage
nvidia-smi  # Monitor GPU usage (if available)

# Reduce simulation complexity
# - Lower physics update rate
# - Reduce sensor frequencies
# - Simplify model geometry
```

### Connection Issues

```bash
# Check Gazebo topics
gz topic -l

# Check ROS 2 topics
ros2 topic list

# Verify bridge is running
ros2 node list | grep bridge
```

## Practical Exercise

### Create a Robot Obstacle Course

**Goal**: Build a simulation with a robot navigating obstacles

**Steps**:
1. Create world with obstacles
2. Add a wheeled robot with sensors
3. Implement basic obstacle avoidance
4. Test in different scenarios

**Expected Learning**:
- World creation skills
- Model integration
- Sensor configuration
- ROS 2 integration

## Key Takeaways

- Gazebo provides realistic physics simulation for robotics
- SDF format defines worlds, models, and sensors
- Plugins extend functionality and enable ROS 2 integration
- Proper configuration balances accuracy and performance
- Command-line tools enable automation and debugging
- Integration with ROS 2 enables complete robot development workflow

---

**Next:** [Lecture 3: URDF and Robot Modeling](./lecture-3.md)