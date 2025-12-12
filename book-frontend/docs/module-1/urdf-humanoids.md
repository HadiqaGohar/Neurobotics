# Understanding URDF (Unified Robot Description Format) for Humanoids

## Introduction to URDF

The Unified Robot Description Format (URDF) is an XML format for describing all aspects of a robot. It is commonly used in ROS (Robot Operating System) to represent a robot's kinematic and dynamic properties, visual appearance, and collision models. While URDF can describe any type of robot, its application to humanoids is particularly important for tasks involving complex movements, balance, and interaction with human-centric environments.

## Why URDF for Humanoids?


Humanoid robots are complex systems with many degrees of freedom, intricate joint structures, and often a need for precise physical modeling. URDF provides a standardized way to define:

*   **Kinematics:** The relationships between joints and links, allowing for forward and inverse kinematics calculations essential for motion planning.
*   **Dynamics:** Mass, inertia, and friction properties of each link, crucial for physics simulation and control.
*   **Visuals:** The graphical representation of the robot, used for visualization in tools like RViz or for rendering in simulators.
*   **Collisions:** Simplified geometric models used to detect collisions in simulations, preventing undesirable self-collisions or collisions with the environment.

## Key Elements of a URDF File

A URDF file is structured around `<robot>` tags, containing `<link>` and `<joint>` elements.

### `<link>` Element

A `link` represents a rigid body segment of the robot. Each link has:

*   **`inertial`:** Defines the mass, center of mass, and inertia tensor of the link. Essential for dynamic simulations.
*   **`visual`:** Describes the visual properties of the link, often by referencing a 3D mesh file (e.g., `.dae`, `.stl`). This is what you see in a simulator or visualization tool.
*   **`collision`:** Provides a simplified geometry for collision detection. This is usually a primitive shape (box, sphere, cylinder) or a coarser mesh than the visual model to reduce computational overhead during collision checks.

**Example Link:**

```xml
<link name="base_link">
  <inertial>
    <mass value="10"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.2 0.2 0.5"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.2 0.5"/>
    </geometry>
  </collision>
</link>
```

### `<joint>` Element

A `joint` describes the connection between two links, defining their relative motion. Key attributes include:

*   **`name`:** A unique identifier for the joint.
*   **`type`:** Specifies the type of joint (e.g., `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`). For humanoids, `revolute` (rotating around an axis) and `prismatic` (linear motion) are common.
*   **`parent` and `child`:** Defines which links the joint connects.
*   **`origin`:** Specifies the transform from the parent link to the joint.
*   **`axis`:** For revolute and prismatic joints, defines the axis of rotation or translation.
*   **`limit`:** Defines the upper and lower limits for joint position, velocity, and effort. Crucial for realistic humanoid movement and preventing self-collision.

**Example Joint:**

```xml
<joint name="torso_to_head" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit effort="100" velocity="10" lower="-1.57" upper="1.57"/>
</joint>
```

## URDF for Humanoid Specifics

When designing URDF for humanoids, special attention is paid to:

*   **Balance and Stability:** Accurate inertial properties are critical for simulations involving balance.
*   **Degrees of Freedom (DoF):** Humanoids typically have many DoFs, requiring careful organization and naming of joints.
*   **Self-Collision:** Defining appropriate collision geometries to avoid parts of the robot colliding with each other during motion.
*   **End Effectors:** Detailed descriptions of hands and grippers for interaction with objects.
*   **Sensors:** Incorporating virtual sensors (e.g., cameras, force sensors) into the URDF for simulation and data acquisition.

## Tools and Visualization

*   **RViz:** A 3D visualization tool in ROS that can display URDF models, sensor data, and planned paths.
*   **Gazebo:** A powerful 3D physics simulator that uses URDF models for realistic robot simulation.
*   **URDF Checkers:** Tools to validate the syntax and structure of URDF files.

## Further Reading

*   ROS URDF Tutorial: [http://wiki.ros.org/urdf/Tutorials](http://wiki.ros.org/urdf/Tutorials)
*   URDF on ROS Wiki: [http://wiki.ros.org/urdf](http://wiki.ros.org/urdf)
