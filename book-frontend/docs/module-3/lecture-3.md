---
sidebar_position: 3
---

# Lecture 3: URDF and Robot Modeling

## What is URDF?

**URDF** (Unified Robot Description Format) is an XML-based format for describing robot models in ROS. It defines the robot's physical structure, including links (rigid bodies), joints (connections), sensors, and visual appearance.

Think of URDF as the **blueprint** of your robot - it tells the computer exactly how your robot is built, how it moves, and what it looks like.

## Why Use URDF?

### Benefits of URDF
- **Standardized format**: Works across all ROS tools
- **Simulation ready**: Direct integration with Gazebo
- **Visualization**: Automatic 3D model generation
- **Kinematics**: Automatic forward/inverse kinematics
- **Modular**: Reusable components and inheritance

### URDF vs Other Formats
| Format | Use Case | Pros | Cons |
|--------|----------|------|------|
| **URDF** | ROS robotics | ROS integration, kinematics | XML verbose, limited physics |
| **SDF** | Gazebo simulation | Advanced physics, sensors | Gazebo-specific |
| **MJCF** | MuJoCo simulation | Fast physics | MuJoCo-specific |
| **STL/OBJ** | 3D models | Visual detail | No kinematics |

## Basic URDF Structure

### Minimal Robot Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link (required) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0" ixz="0" 
               iyy="0.4" iyz="0" izz="0.2"/>
    </inertial>
  </link>
</robot>
```

### Core URDF Elements

#### 1. Robot Tag
```xml
<robot name="my_robot">
  <!-- All robot content goes here -->
</robot>
```

#### 2. Links (Rigid Bodies)
```xml
<link name="link_name">
  <visual>     <!-- How it looks -->
  <collision>  <!-- How it collides -->
  <inertial>   <!-- How it moves (physics) -->
</link>
```

#### 3. Joints (Connections)
```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

## Understanding Links

### Link Components

A link represents a **rigid body** in your robot. Each link has three main components:

#### 1. Visual (Appearance)
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
    <!-- OR -->
    <cylinder radius="0.5" length="1"/>
    <!-- OR -->
    <sphere radius="0.5"/>
    <!-- OR -->
    <mesh filename="package://my_robot/meshes/part.stl"/>
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

#### 2. Collision (Physics)
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>  <!-- Usually simpler than visual -->
  </geometry>
</collision>
```

#### 3. Inertial (Mass Properties)
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.1" ixy="0" ixz="0"
           iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

### Coordinate Frames and Origins

Every element in URDF has an **origin** that defines its position and orientation:

```xml
<origin xyz="x y z" rpy="roll pitch yaw"/>
```

- **xyz**: Position in meters (x=forward, y=left, z=up)
- **rpy**: Rotation in radians (roll=x-axis, pitch=y-axis, yaw=z-axis)

### Material Definitions

```xml
<!-- Define materials at robot level -->
<material name="blue">
  <color rgba="0 0 1 1"/>  <!-- Red Green Blue Alpha -->
</material>

<material name="silver">
  <color rgba="0.7 0.7 0.7 1"/>
</material>

<!-- Use in links -->
<visual>
  <geometry>...</geometry>
  <material name="blue"/>
</visual>
```

## Understanding Joints

Joints connect links and define how they can move relative to each other.

### Joint Types

#### 1. Fixed Joint
```xml
<joint name="base_to_sensor" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_link"/>
  <origin xyz="0.3 0 0.2" rpy="0 0 0"/>
</joint>
```
**Use case**: Attach sensors, cameras, or fixed components

#### 2. Revolute Joint (Rotating)
```xml
<joint name="wheel_joint" type="revolute">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0 0.2 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotation axis -->
  <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
</joint>
```
**Use case**: Wheels, rotating joints with limits

#### 3. Continuous Joint (Unlimited Rotation)
```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0 0.2 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit effort="100" velocity="10"/>  <!-- No position limits -->
</joint>
```
**Use case**: Wheels, propellers, continuous rotation

#### 4. Prismatic Joint (Linear)
```xml
<joint name="linear_actuator" type="prismatic">
  <parent link="base_link"/>
  <child link="slider_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Translation axis -->
  <limit lower="0" upper="0.5" effort="1000" velocity="1"/>
</joint>
```
**Use case**: Linear actuators, telescoping parts

#### 5. Planar Joint (2D Movement)
```xml
<joint name="mobile_base" type="planar">
  <parent link="world"/>
  <child link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```
**Use case**: Mobile robots moving on flat surfaces

#### 6. Floating Joint (6DOF)
```xml
<joint name="free_floating" type="floating">
  <parent link="world"/>
  <child link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```
**Use case**: Flying robots, free-floating objects

### Joint Properties

#### Limits
```xml
<limit lower="-1.57" upper="1.57"    <!-- Position limits (rad or m) -->
       effort="100"                   <!-- Max force/torque -->
       velocity="10"/>                <!-- Max velocity -->
```

#### Dynamics
```xml
<dynamics damping="0.1" friction="0.05"/>
```

#### Safety Controller
```xml
<safety_controller soft_lower_limit="-1.5" 
                   soft_upper_limit="1.5"
                   k_position="100" 
                   k_velocity="10"/>
```

## Building a Complete Robot

Let's build a simple mobile robot step by step:

### Step 1: Base Structure

```xml
<?xml version="1.0"?>
<robot name="mobile_robot">
  
  <!-- Materials -->
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  
  <material name="black">
    <color rgba="0.2 0.2 0.2 1"/>
  </material>
  
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0" ixz="0"
               iyy="0.6" iyz="0" izz="0.8"/>
    </inertial>
  </link>
</robot>
```

### Step 2: Add Wheels

```xml
  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0"
               iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>
  
  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0"
               iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>
```

### Step 3: Add Wheel Joints

```xml
  <!-- Left Wheel Joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.225 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="100" velocity="10"/>
  </joint>
  
  <!-- Right Wheel Joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.225 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="100" velocity="10"/>
  </joint>
```

### Step 4: Add Caster Wheel

```xml
  <!-- Caster Wheel -->
  <link name="caster_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Caster Joint -->
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.2 0 0.05" rpy="0 0 0"/>
  </joint>
```

### Step 5: Add Sensor

```xml
  <!-- LIDAR Sensor -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="red"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  
  <!-- LIDAR Joint -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>
```

## Calculating Inertia Properties

### Why Inertia Matters
Inertia determines how objects respond to forces and torques. Incorrect inertia can cause:
- Unrealistic simulation behavior
- Instability in control
- Poor performance

### Basic Inertia Formulas

#### Box (Rectangular)
```python
def box_inertia(mass, length, width, height):
    ixx = (mass / 12) * (width**2 + height**2)
    iyy = (mass / 12) * (length**2 + height**2)
    izz = (mass / 12) * (length**2 + width**2)
    return ixx, iyy, izz

# Example: 10kg box, 0.6m x 0.4m x 0.2m
ixx, iyy, izz = box_inertia(10, 0.6, 0.4, 0.2)
# ixx = 0.167, iyy = 0.333, izz = 0.433
```

#### Cylinder
```python
def cylinder_inertia(mass, radius, height):
    ixx = iyy = (mass / 12) * (3 * radius**2 + height**2)
    izz = (mass / 2) * radius**2
    return ixx, iyy, izz

# Example: 1kg wheel, radius=0.1m, height=0.05m
ixx, iyy, izz = cylinder_inertia(1, 0.1, 0.05)
# ixx = iyy = 0.0271, izz = 0.005
```

#### Sphere
```python
def sphere_inertia(mass, radius):
    inertia = (2/5) * mass * radius**2
    return inertia, inertia, inertia

# Example: 0.5kg sphere, radius=0.05m
ixx, iyy, izz = sphere_inertia(0.5, 0.05)
# All = 0.0005
```

## Working with Xacro

**Xacro** (XML Macros) makes URDF more manageable by adding:
- Variables and parameters
- Mathematical expressions
- Macros and functions
- File inclusion

### Basic Xacro Example

```xml
<?xml version="1.0"?>
<robot name="mobile_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Parameters -->
  <xacro:property name="base_width" value="0.4"/>
  <xacro:property name="base_length" value="0.6"/>
  <xacro:property name="base_height" value="0.2"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  
  <!-- Wheel Macro -->
  <xacro:macro name="wheel" params="prefix y_reflect">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="1.57 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black"/>
      </visual>
      
      <collision>
        <origin xyz="0 0 0" rpy="1.57 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      
      <inertial>
        <origin xyz="0 0 0" rpy="1.57 0 0"/>
        <mass value="1.0"/>
        <xacro:cylinder_inertia mass="1.0" radius="${wheel_radius}" height="${wheel_width}"/>
      </inertial>
    </link>
    
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${y_reflect * (base_width/2 + wheel_width/2)} ${wheel_radius}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit effort="100" velocity="10"/>
    </joint>
  </xacro:macro>
  
  <!-- Inertia Macro -->
  <xacro:macro name="cylinder_inertia" params="mass radius height">
    <inertia ixx="${mass * (3*radius*radius + height*height) / 12}"
             ixy="0" ixz="0"
             iyy="${mass * (3*radius*radius + height*height) / 12}"
             iyz="0"
             izz="${mass * radius * radius / 2}"/>
  </xacro:macro>
  
  <!-- Use macros -->
  <xacro:wheel prefix="left" y_reflect="1"/>
  <xacro:wheel prefix="right" y_reflect="-1"/>
  
</robot>
```

### Converting Xacro to URDF

```bash
# Convert xacro to URDF
ros2 run xacro xacro robot.urdf.xacro > robot.urdf

# Or with parameters
ros2 run xacro xacro robot.urdf.xacro wheel_radius:=0.15 > robot.urdf
```

## Visualizing Your Robot

### Using RViz

```bash
# Install joint state publisher
sudo apt install ros-humble-joint-state-publisher-gui

# Launch visualization
ros2 launch urdf_tutorial display.launch.py model:=robot.urdf
```

### Launch File for Visualization

```python
# display.launch.py
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('my_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf_file}]
        ),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot.rviz')]
        )
    ])
```

## Common URDF Mistakes and Solutions

### 1. Missing Base Link
**Problem**: No link named "base_link"
**Solution**: Always have a base_link as root

### 2. Incorrect Inertia
**Problem**: Zero or unrealistic inertia values
**Solution**: Use proper inertia calculations

### 3. Collision vs Visual Mismatch
**Problem**: Complex visual geometry for collision
**Solution**: Use simplified collision geometry

```xml
<!-- Good: Simple collision -->
<collision>
  <geometry>
    <box size="0.6 0.4 0.2"/>  <!-- Simple box -->
  </geometry>
</collision>

<visual>
  <geometry>
    <mesh filename="complex_model.stl"/>  <!-- Detailed visual -->
  </geometry>
</visual>
```

### 4. Wrong Joint Axes
**Problem**: Joints rotating in wrong direction
**Solution**: Check axis definition

```xml
<!-- Wheel should rotate around Y-axis -->
<axis xyz="0 1 0"/>  <!-- Correct -->
<axis xyz="1 0 0"/>  <!-- Wrong -->
```

### 5. Scale Issues
**Problem**: Robot too big/small in simulation
**Solution**: Use realistic dimensions (meters)

## Validation and Testing

### Check URDF Syntax

```bash
# Check for errors
check_urdf robot.urdf

# Get model info
urdf_to_graphiz robot.urdf
```

### Test in Gazebo

```bash
# Spawn in Gazebo
gz sim empty.sdf
gz service -s /world/empty/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "robot.urdf"'
```

## Best Practices

### 1. Naming Conventions
- Use descriptive names: `left_wheel`, not `link1`
- Follow ROS conventions: `base_link`, `odom`
- Be consistent across similar components

### 2. Coordinate Frames
- **X-axis**: Forward direction
- **Y-axis**: Left direction  
- **Z-axis**: Up direction
- Place joint origins at rotation centers

### 3. Mass Distribution
- Concentrate mass in main body
- Use realistic mass values
- Don't forget wheel masses

### 4. Modularity
- Use Xacro macros for repeated components
- Separate visual and collision meshes
- Parameterize dimensions

### 5. Documentation
```xml
<!-- Document your robot structure -->
<!-- 
  Mobile Robot URDF
  - Base: 0.6m x 0.4m x 0.2m, 10kg
  - Wheels: 0.1m radius, differential drive
  - Sensors: LIDAR on top
-->
```

## Practical Exercise

### Build Your Own Robot

**Goal**: Create a URDF for a custom robot

**Requirements**:
1. Mobile base with differential drive
2. At least one sensor (camera or LIDAR)
3. Proper inertia calculations
4. Xacro macros for wheels
5. Visualization in RViz

**Steps**:
1. Design robot structure on paper
2. Create basic URDF with base link
3. Add wheels with proper joints
4. Add sensors and other components
5. Calculate and add inertia properties
6. Convert to Xacro for modularity
7. Test visualization and physics

## Key Takeaways

- URDF defines robot structure for ROS ecosystem
- Links represent rigid bodies with visual, collision, and inertial properties
- Joints connect links and define motion constraints
- Proper inertia calculation is crucial for realistic simulation
- Xacro adds programming features to URDF
- Always validate URDF syntax and test in simulation
- Follow naming conventions and coordinate frame standards
- Use modular design with macros for complex robots

---

**Next:** [Lecture 4: Physics Simulation and Environments](./lecture-4.md)