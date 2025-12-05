---
sidebar_position: 3
---

# Lecture 3: Sensor Fusion and State Estimation

## Introduction to Sensor Fusion

**Sensor Fusion** is the process of combining data from multiple sensors to produce more accurate, reliable, and complete information than any single sensor could provide alone. In robotics, sensor fusion is essential for robust perception and navigation.

## Why Sensor Fusion?

### Individual Sensor Limitations

| Sensor Type | Strengths | Weaknesses |
|-------------|-----------|------------|
| **Camera** | Rich visual information, color | No depth, lighting dependent |
| **LIDAR** | Precise distance, works in dark | Expensive, limited texture info |
| **IMU** | High frequency, orientation | Drift over time, no position |
| **GPS** | Global position | Indoor/urban limitations |
| **Encoders** | Precise relative motion | Accumulates error, slippage |

### Benefits of Fusion

#### 1. Complementary Information
```python
# Example: Camera + LIDAR fusion
camera_data = {
    'objects': ['car', 'person', 'tree'],
    'colors': ['red', 'blue', 'green'],
    'textures': ['smooth', 'clothing', 'bark']
}

lidar_data = {
    'distances': [5.2, 3.1, 8.7],  # meters
    'positions': [(5.0, 1.2), (2.8, -0.5), (8.1, 2.3)],
    'sizes': [(1.8, 4.2), (0.6, 1.8), (0.8, 12.0)]  # width, height
}

# Fused result provides both semantic and geometric information
fused_result = {
    'red_car': {'distance': 5.2, 'position': (5.0, 1.2), 'size': (1.8, 4.2)},
    'person': {'distance': 3.1, 'position': (2.8, -0.5), 'size': (0.6, 1.8)},
    'tree': {'distance': 8.7, 'position': (8.1, 2.3), 'size': (0.8, 12.0)}
}
```

#### 2. Redundancy and Reliability
```python
# Multiple sensors measuring the same quantity
gps_position = (10.5, 20.3)      # ±3m accuracy
visual_odometry = (10.2, 20.1)   # ±0.5m accuracy (short term)
wheel_odometry = (10.8, 19.9)    # ±0.2m accuracy (very short term)

# Fused position is more reliable than any single measurement
fused_position = weighted_average([gps_position, visual_odometry, wheel_odometry],
                                weights=[0.3, 0.4, 0.3])
```

#### 3. Improved Accuracy
- **Statistical combination** reduces random errors
- **Cross-validation** detects and corrects systematic errors
- **Temporal consistency** smooths noisy measurements

## State Estimation Fundamentals

### State Representation

```python
# Robot state vector
robot_state = {
    'position': [x, y, z],           # 3D position (m)
    'orientation': [roll, pitch, yaw], # Euler angles (rad)
    'linear_velocity': [vx, vy, vz],  # Linear velocity (m/s)
    'angular_velocity': [wx, wy, wz], # Angular velocity (rad/s)
    'linear_acceleration': [ax, ay, az], # Linear acceleration (m/s²)
    'timestamp': 1634567890.123       # Time (seconds)
}

# State vector for Kalman filter (12-dimensional)
state_vector = np.array([x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz])
```

### Uncertainty Representation

```python
import numpy as np

# Covariance matrix represents uncertainty
# Diagonal elements: variance of each state variable
# Off-diagonal elements: correlations between variables

covariance_matrix = np.array([
    # x    y    z   roll pitch yaw   vx   vy   vz   wx   wy   wz
    [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # x
    [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # y
    [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # z
    [0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # roll
    [0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # pitch
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # yaw
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0],  # vx
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0],  # vy
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0],  # vz
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0],  # wx
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0],  # wy
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]   # wz
])
```

## Kalman Filter

The **Kalman Filter** is the most widely used algorithm for linear state estimation and sensor fusion.

### Basic Kalman Filter

```python
import numpy as np

class KalmanFilter:
    def __init__(self, state_dim, measurement_dim):
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim
        
        # State vector and covariance
        self.x = np.zeros(state_dim)  # State estimate
        self.P = np.eye(state_dim)    # State covariance
        
        # System matrices
        self.F = np.eye(state_dim)    # State transition model
        self.H = np.eye(measurement_dim, state_dim)  # Observation model
        self.Q = np.eye(state_dim)    # Process noise covariance
        self.R = np.eye(measurement_dim)  # Measurement noise covariance
        
    def predict(self, dt):
        """Prediction step"""
        # Update state transition matrix with time step
        self.update_state_transition(dt)
        
        # Predict state
        self.x = np.dot(self.F, self.x)
        
        # Predict covariance
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        
    def update(self, measurement):
        """Update step with measurement"""
        # Innovation (measurement residual)
        y = measurement - np.dot(self.H, self.x)
        
        # Innovation covariance
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        
        # Kalman gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # Update state estimate
        self.x = self.x + np.dot(K, y)
        
        # Update covariance
        I = np.eye(self.state_dim)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)
        
    def update_state_transition(self, dt):
        """Update state transition matrix for constant velocity model"""
        # For position-velocity model: x_k+1 = x_k + v_k * dt
        self.F = np.array([
            [1, 0, dt, 0],   # x position
            [0, 1, 0, dt],   # y position  
            [0, 0, 1, 0],    # x velocity
            [0, 0, 0, 1]     # y velocity
        ])

# Example usage for 2D robot tracking
def track_robot_2d():
    # Initialize filter (position and velocity in 2D)
    kf = KalmanFilter(state_dim=4, measurement_dim=2)
    
    # Initial state: [x, y, vx, vy]
    kf.x = np.array([0.0, 0.0, 0.0, 0.0])
    
    # Process noise (model uncertainty)
    kf.Q = np.diag([0.1, 0.1, 0.5, 0.5])
    
    # Measurement noise (sensor uncertainty)
    kf.R = np.diag([0.5, 0.5])  # GPS measurement noise
    
    # Observation matrix (we measure position only)
    kf.H = np.array([
        [1, 0, 0, 0],  # Measure x position
        [0, 1, 0, 0]   # Measure y position
    ])
    
    # Simulation
    dt = 0.1  # 10 Hz
    measurements = [(1.0, 0.5), (2.1, 1.2), (3.0, 1.8)]  # GPS measurements
    
    for measurement in measurements:
        # Predict
        kf.predict(dt)
        
        # Update with measurement
        kf.update(np.array(measurement))
        
        print(f"State: {kf.x}")
        print(f"Position uncertainty: {np.sqrt(kf.P[0,0]):.3f}, {np.sqrt(kf.P[1,1]):.3f}")
```

### Extended Kalman Filter (EKF)

For nonlinear systems, we use the **Extended Kalman Filter**:

```python
import numpy as np
from scipy.linalg import inv

class ExtendedKalmanFilter:
    def __init__(self, state_dim, measurement_dim):
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim
        
        # State and covariance
        self.x = np.zeros(state_dim)
        self.P = np.eye(state_dim)
        
        # Noise covariances
        self.Q = np.eye(state_dim)
        self.R = np.eye(measurement_dim)
        
    def predict(self, control_input, dt):
        """Prediction step with nonlinear motion model"""
        # Nonlinear state transition
        self.x = self.motion_model(self.x, control_input, dt)
        
        # Jacobian of motion model
        F = self.motion_jacobian(self.x, control_input, dt)
        
        # Predict covariance
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q
        
    def update(self, measurement):
        """Update step with nonlinear measurement model"""
        # Predicted measurement
        z_pred = self.measurement_model(self.x)
        
        # Jacobian of measurement model
        H = self.measurement_jacobian(self.x)
        
        # Innovation
        y = measurement - z_pred
        
        # Innovation covariance
        S = np.dot(np.dot(H, self.P), H.T) + self.R
        
        # Kalman gain
        K = np.dot(np.dot(self.P, H.T), inv(S))
        
        # Update state and covariance
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.state_dim)
        self.P = np.dot((I - np.dot(K, H)), self.P)
        
    def motion_model(self, state, control, dt):
        """Nonlinear motion model for differential drive robot"""
        x, y, theta, v, omega = state
        
        # Control inputs
        v_cmd, omega_cmd = control
        
        # Motion equations
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + omega * dt
        v_new = v_cmd  # Assume instantaneous velocity control
        omega_new = omega_cmd
        
        return np.array([x_new, y_new, theta_new, v_new, omega_new])
    
    def motion_jacobian(self, state, control, dt):
        """Jacobian of motion model"""
        x, y, theta, v, omega = state
        
        F = np.array([
            [1, 0, -v * np.sin(theta) * dt, np.cos(theta) * dt, 0],
            [0, 1,  v * np.cos(theta) * dt, np.sin(theta) * dt, 0],
            [0, 0,  1,                      0,                   dt],
            [0, 0,  0,                      1,                   0],
            [0, 0,  0,                      0,                   1]
        ])
        
        return F
    
    def measurement_model(self, state):
        """Measurement model (e.g., GPS + compass)"""
        x, y, theta, v, omega = state
        
        # Measure position and orientation
        return np.array([x, y, theta])
    
    def measurement_jacobian(self, state):
        """Jacobian of measurement model"""
        H = np.array([
            [1, 0, 0, 0, 0],  # x measurement
            [0, 1, 0, 0, 0],  # y measurement
            [0, 0, 1, 0, 0]   # theta measurement
        ])
        
        return H
```

## IMU Integration and Orientation Estimation

### IMU Data Processing

```python
import numpy as np
from scipy.spatial.transform import Rotation

class IMUProcessor:
    def __init__(self):
        # Calibration parameters
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.accel_scale = np.ones(3)
        self.gyro_scale = np.ones(3)
        
        # Current state
        self.orientation = Rotation.identity()
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        
        # Gravity vector
        self.gravity = np.array([0, 0, -9.81])
        
    def calibrate_static(self, accel_samples, gyro_samples):
        """Calibrate IMU using static samples"""
        # Gyroscope bias (should be zero when stationary)
        self.gyro_bias = np.mean(gyro_samples, axis=0)
        
        # Accelerometer bias (should measure gravity when stationary)
        accel_mean = np.mean(accel_samples, axis=0)
        gravity_magnitude = np.linalg.norm(self.gravity)
        measured_magnitude = np.linalg.norm(accel_mean)
        
        # Scale factor to match gravity
        self.accel_scale = np.ones(3) * (gravity_magnitude / measured_magnitude)
        
        # Bias is the difference from expected gravity vector
        expected_gravity = np.array([0, 0, gravity_magnitude])
        self.accel_bias = accel_mean - expected_gravity
        
    def process_imu_data(self, accel_raw, gyro_raw, dt):
        """Process raw IMU data"""
        # Apply calibration
        accel = (accel_raw - self.accel_bias) * self.accel_scale
        gyro = (gyro_raw - self.gyro_bias) * self.gyro_scale
        
        # Update orientation using gyroscope
        self.update_orientation(gyro, dt)
        
        # Update velocity and position using accelerometer
        self.update_motion(accel, dt)
        
        return {
            'orientation': self.orientation,
            'velocity': self.velocity,
            'position': self.position,
            'accel_corrected': accel,
            'gyro_corrected': gyro
        }
    
    def update_orientation(self, gyro, dt):
        """Update orientation using gyroscope data"""
        # Angular velocity vector
        angular_velocity = gyro
        
        # Rotation angle
        angle = np.linalg.norm(angular_velocity) * dt
        
        if angle > 1e-8:  # Avoid division by zero
            # Rotation axis
            axis = angular_velocity / np.linalg.norm(angular_velocity)
            
            # Create rotation
            delta_rotation = Rotation.from_rotvec(axis * angle)
            
            # Update orientation
            self.orientation = self.orientation * delta_rotation
    
    def update_motion(self, accel, dt):
        """Update velocity and position using accelerometer"""
        # Remove gravity from acceleration
        gravity_world = self.orientation.apply(self.gravity)
        accel_world = self.orientation.apply(accel) - gravity_world
        
        # Update velocity (integrate acceleration)
        self.velocity += accel_world * dt
        
        # Update position (integrate velocity)
        self.position += self.velocity * dt + 0.5 * accel_world * dt**2
```

### Complementary Filter

A **Complementary Filter** is a simple but effective way to fuse IMU data:

```python
class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha  # Trust gyroscope more (0.98 = 98% gyro, 2% accel)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
    def update(self, accel, gyro, dt):
        """Update orientation using complementary filter"""
        # Calculate angles from accelerometer
        accel_roll = np.arctan2(accel[1], accel[2])
        accel_pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
        
        # Integrate gyroscope
        gyro_roll = self.roll + gyro[0] * dt
        gyro_pitch = self.pitch + gyro[1] * dt
        gyro_yaw = self.yaw + gyro[2] * dt
        
        # Complementary filter
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        self.yaw = gyro_yaw  # No accelerometer correction for yaw
        
        return self.roll, self.pitch, self.yaw
```

## Multi-Sensor Fusion Architecture

### Centralized Fusion

```python
class CentralizedSensorFusion:
    def __init__(self):
        # Initialize EKF for robot state
        self.ekf = ExtendedKalmanFilter(state_dim=9, measurement_dim=6)
        
        # State: [x, y, z, roll, pitch, yaw, vx, vy, vz]
        self.ekf.x = np.zeros(9)
        
        # Sensor data buffers
        self.imu_buffer = []
        self.gps_buffer = []
        self.lidar_buffer = []
        self.camera_buffer = []
        
        # Sensor noise models
        self.sensor_noise = {
            'imu': {'accel': 0.1, 'gyro': 0.01},
            'gps': {'position': 3.0},
            'lidar': {'range': 0.05},
            'camera': {'pixel': 1.0}
        }
        
    def add_imu_measurement(self, accel, gyro, timestamp):
        """Add IMU measurement to buffer"""
        self.imu_buffer.append({
            'accel': accel,
            'gyro': gyro,
            'timestamp': timestamp,
            'type': 'imu'
        })
        
    def add_gps_measurement(self, position, timestamp):
        """Add GPS measurement to buffer"""
        self.gps_buffer.append({
            'position': position,
            'timestamp': timestamp,
            'type': 'gps'
        })
        
    def add_lidar_measurement(self, ranges, angles, timestamp):
        """Add LIDAR measurement to buffer"""
        self.lidar_buffer.append({
            'ranges': ranges,
            'angles': angles,
            'timestamp': timestamp,
            'type': 'lidar'
        })
        
    def process_measurements(self, current_time):
        """Process all measurements up to current time"""
        # Collect all measurements
        all_measurements = []
        all_measurements.extend(self.imu_buffer)
        all_measurements.extend(self.gps_buffer)
        all_measurements.extend(self.lidar_buffer)
        
        # Sort by timestamp
        all_measurements.sort(key=lambda x: x['timestamp'])
        
        # Process measurements in chronological order
        last_time = current_time - 0.1  # Process last 100ms
        
        for measurement in all_measurements:
            if measurement['timestamp'] <= current_time:
                self.process_single_measurement(measurement, last_time)
                last_time = measurement['timestamp']
        
        # Clear processed measurements
        self.clear_old_measurements(current_time)
        
    def process_single_measurement(self, measurement, last_time):
        """Process a single measurement"""
        dt = measurement['timestamp'] - last_time
        
        if measurement['type'] == 'imu':
            # Use IMU for prediction step
            control_input = np.array([0, 0])  # No control input
            self.ekf.predict(control_input, dt)
            
            # Update with IMU measurement (orientation)
            orientation_measurement = self.imu_to_orientation(
                measurement['accel'], measurement['gyro'])
            self.ekf.update(orientation_measurement)
            
        elif measurement['type'] == 'gps':
            # Update with GPS position
            position_measurement = measurement['position']
            self.ekf.update(position_measurement)
            
        elif measurement['type'] == 'lidar':
            # Extract position from LIDAR scan matching
            position_update = self.lidar_scan_matching(
                measurement['ranges'], measurement['angles'])
            if position_update is not None:
                self.ekf.update(position_update)
    
    def imu_to_orientation(self, accel, gyro):
        """Convert IMU data to orientation measurement"""
        # Simple conversion (in practice, use more sophisticated method)
        roll = np.arctan2(accel[1], accel[2])
        pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
        
        return np.array([roll, pitch, 0])  # No yaw from accelerometer
    
    def lidar_scan_matching(self, ranges, angles):
        """Perform scan matching to get position update"""
        # Placeholder for scan matching algorithm
        # In practice, this would use ICP or other scan matching
        return None
    
    def clear_old_measurements(self, current_time):
        """Remove old measurements from buffers"""
        cutoff_time = current_time - 1.0  # Keep 1 second of history
        
        self.imu_buffer = [m for m in self.imu_buffer if m['timestamp'] > cutoff_time]
        self.gps_buffer = [m for m in self.gps_buffer if m['timestamp'] > cutoff_time]
        self.lidar_buffer = [m for m in self.lidar_buffer if m['timestamp'] > cutoff_time]
    
    def get_current_state(self):
        """Get current robot state estimate"""
        return {
            'position': self.ekf.x[:3],
            'orientation': self.ekf.x[3:6],
            'velocity': self.ekf.x[6:9],
            'covariance': self.ekf.P
        }
```

## ROS 2 Integration

### Sensor Fusion Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Initialize fusion system
        self.fusion = CentralizedSensorFusion()
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/pose/filtered', 10)
        
        # Timer for processing
        self.timer = self.create_timer(0.01, self.process_fusion)  # 100 Hz
        
        self.get_logger().info('Sensor Fusion Node initialized')
    
    def imu_callback(self, msg):
        """Process IMU data"""
        # Extract acceleration and angular velocity
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Add to fusion system
        self.fusion.add_imu_measurement(accel, gyro, timestamp)
    
    def gps_callback(self, msg):
        """Process GPS data"""
        if msg.status.status >= 0:  # Valid GPS fix
            # Convert lat/lon to local coordinates (simplified)
            position = np.array([
                msg.latitude * 111320,   # Rough conversion to meters
                msg.longitude * 111320,
                msg.altitude
            ])
            
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # Add to fusion system
            self.fusion.add_gps_measurement(position, timestamp)
    
    def scan_callback(self, msg):
        """Process LIDAR scan data"""
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, 
                          msg.angle_increment)
        
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Add to fusion system
        self.fusion.add_lidar_measurement(ranges, angles, timestamp)
    
    def process_fusion(self):
        """Process sensor fusion and publish results"""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Process all measurements
        self.fusion.process_measurements(current_time)
        
        # Get current state estimate
        state = self.fusion.get_current_state()
        
        # Publish odometry
        self.publish_odometry(state, current_time)
        
        # Publish pose
        self.publish_pose(state, current_time)
    
    def publish_odometry(self, state, timestamp):
        """Publish odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = float(state['position'][0])
        odom_msg.pose.pose.position.y = float(state['position'][1])
        odom_msg.pose.pose.position.z = float(state['position'][2])
        
        # Orientation (convert from Euler to quaternion)
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(
            state['orientation'][0],  # roll
            state['orientation'][1],  # pitch
            state['orientation'][2]   # yaw
        )
        
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Velocity
        odom_msg.twist.twist.linear.x = float(state['velocity'][0])
        odom_msg.twist.twist.linear.y = float(state['velocity'][1])
        odom_msg.twist.twist.linear.z = float(state['velocity'][2])
        
        # Covariance (simplified)
        pose_cov = np.zeros(36)
        pose_cov[0] = state['covariance'][0, 0]   # x
        pose_cov[7] = state['covariance'][1, 1]   # y
        pose_cov[14] = state['covariance'][2, 2]  # z
        pose_cov[21] = state['covariance'][3, 3]  # roll
        pose_cov[28] = state['covariance'][4, 4]  # pitch
        pose_cov[35] = state['covariance'][5, 5]  # yaw
        
        odom_msg.pose.covariance = pose_cov.tolist()
        
        self.odom_pub.publish(odom_msg)
    
    def publish_pose(self, state, timestamp):
        """Publish pose with covariance"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Position
        pose_msg.pose.pose.position.x = float(state['position'][0])
        pose_msg.pose.pose.position.y = float(state['position'][1])
        pose_msg.pose.pose.position.z = float(state['position'][2])
        
        # Orientation
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(
            state['orientation'][0],
            state['orientation'][1],
            state['orientation'][2]
        )
        
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]
        
        # Covariance
        pose_cov = np.zeros(36)
        pose_cov[0] = state['covariance'][0, 0]   # x
        pose_cov[7] = state['covariance'][1, 1]   # y
        pose_cov[14] = state['covariance'][2, 2]  # z
        pose_cov[21] = state['covariance'][3, 3]  # roll
        pose_cov[28] = state['covariance'][4, 4]  # pitch
        pose_cov[35] = state['covariance'][5, 5]  # yaw
        
        pose_msg.pose.covariance = pose_cov.tolist()
        
        self.pose_pub.publish(pose_msg)

def main():
    rclpy.init()
    fusion_node = SensorFusionNode()
    
    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Fusion Techniques

### Particle Filter

```python
import numpy as np

class ParticleFilter:
    def __init__(self, num_particles=1000, state_dim=3):
        self.num_particles = num_particles
        self.state_dim = state_dim
        
        # Initialize particles
        self.particles = np.random.randn(num_particles, state_dim)
        self.weights = np.ones(num_particles) / num_particles
        
    def predict(self, control_input, process_noise):
        """Prediction step"""
        for i in range(self.num_particles):
            # Apply motion model with noise
            self.particles[i] = self.motion_model(
                self.particles[i], control_input) + np.random.multivariate_normal(
                np.zeros(self.state_dim), process_noise)
    
    def update(self, measurement, measurement_noise):
        """Update step"""
        for i in range(self.num_particles):
            # Calculate likelihood of measurement given particle state
            predicted_measurement = self.measurement_model(self.particles[i])
            
            # Gaussian likelihood
            diff = measurement - predicted_measurement
            likelihood = np.exp(-0.5 * np.dot(diff, np.linalg.solve(measurement_noise, diff)))
            
            # Update weight
            self.weights[i] *= likelihood
        
        # Normalize weights
        self.weights /= np.sum(self.weights)
        
        # Resample if effective sample size is low
        if self.effective_sample_size() < self.num_particles / 2:
            self.resample()
    
    def resample(self):
        """Resample particles based on weights"""
        indices = np.random.choice(
            self.num_particles, self.num_particles, p=self.weights)
        
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles
    
    def effective_sample_size(self):
        """Calculate effective sample size"""
        return 1.0 / np.sum(self.weights**2)
    
    def get_state_estimate(self):
        """Get weighted average state estimate"""
        return np.average(self.particles, weights=self.weights, axis=0)
    
    def motion_model(self, state, control):
        """Motion model (to be implemented for specific robot)"""
        return state  # Placeholder
    
    def measurement_model(self, state):
        """Measurement model (to be implemented for specific sensors)"""
        return state  # Placeholder
```

## Key Takeaways

- Sensor fusion combines multiple sensors to improve accuracy, reliability, and robustness
- Kalman filters are optimal for linear systems, while Extended Kalman Filters handle nonlinear systems
- IMU integration requires careful handling of biases, noise, and coordinate transformations
- Complementary filters provide a simple but effective way to fuse IMU data
- Centralized fusion architectures process all sensor data in a single estimator
- ROS 2 integration enables real-time sensor fusion in robot systems
- Particle filters can handle highly nonlinear systems and non-Gaussian noise
- Proper uncertainty modeling and noise characterization are crucial for effective fusion
- Time synchronization and measurement ordering are important for multi-sensor systems

---

**Next:** [Lecture 4: NVIDIA Isaac and AI Acceleration](./lecture-4.md)