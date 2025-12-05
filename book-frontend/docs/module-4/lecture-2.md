---
sidebar_position: 2
---

# Lecture 2: LIDAR and Point Cloud Processing

## Introduction to LIDAR

**LIDAR** (Light Detection and Ranging) is a remote sensing technology that uses laser light to measure distances and create detailed 3D maps of the environment. For robots, LIDAR provides precise distance measurements and is essential for navigation, mapping, and obstacle avoidance.

## How LIDAR Works

### Basic Principle

```
Laser Emitter → Light Pulse → Object → Reflected Light → Sensor
Time of Flight = Distance Calculation
Distance = (Speed of Light × Time) / 2
```

### LIDAR Components

#### 1. Laser Source
- **Wavelength**: Typically 905nm or 1550nm infrared
- **Power**: Eye-safe levels (Class 1 laser)
- **Pulse rate**: Up to millions of pulses per second

#### 2. Detector
- **Photodiode**: Converts light to electrical signal
- **Timing circuits**: Measure time of flight precisely
- **Signal processing**: Filter noise and extract distance

#### 3. Scanning Mechanism
- **Rotating mirror**: Mechanical scanning
- **MEMS mirrors**: Micro-electromechanical scanning
- **Phased arrays**: Electronic beam steering (solid-state)

## Types of LIDAR

### 1. 2D LIDAR (Planar)

```python
# 2D LIDAR data structure
lidar_2d_data = {
    'angle_min': -3.14159,      # Start angle (radians)
    'angle_max': 3.14159,       # End angle (radians)
    'angle_increment': 0.0175,  # Angular resolution (radians)
    'range_min': 0.1,           # Minimum range (meters)
    'range_max': 30.0,          # Maximum range (meters)
    'ranges': [1.2, 1.5, 2.1, ...],  # Distance measurements
    'intensities': [100, 95, 120, ...]  # Reflection intensities
}
```

**Applications:**
- Mobile robot navigation
- 2D mapping and localization
- Obstacle detection
- Perimeter monitoring

**Popular 2D LIDAR Sensors:**
- **Hokuyo UST-10LX**: 10m range, 270° field of view
- **SICK TiM3xx**: 4m range, 270° field of view
- **RPLidar A1**: 12m range, 360° scanning
- **Velodyne VLP-16 Lite**: 16-beam 2D mode

### 2. 3D LIDAR (Volumetric)

```python
# 3D LIDAR point cloud structure
point_cloud_data = {
    'header': {
        'timestamp': 1634567890.123,
        'frame_id': 'lidar_link'
    },
    'points': [
        {'x': 1.2, 'y': 0.5, 'z': 0.1, 'intensity': 100},
        {'x': 1.5, 'y': 0.3, 'z': 0.2, 'intensity': 95},
        # ... thousands of points
    ],
    'width': 1024,    # Points per scan line
    'height': 64,     # Number of scan lines
    'is_dense': False # Contains invalid points
}
```

**Applications:**
- Autonomous vehicles
- 3D mapping and reconstruction
- Object recognition and tracking
- Terrain analysis

**Popular 3D LIDAR Sensors:**
- **Velodyne VLP-16**: 16 beams, 100m range
- **Ouster OS1-64**: 64 beams, 120m range
- **Livox Horizon**: Non-repetitive scanning pattern
- **Hesai PandarXT**: 32 beams, automotive grade

## Point Cloud Data Structures

### Point Cloud Library (PCL) Format

```cpp
// C++ PCL point types
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Basic XYZ point
struct PointXYZ {
    float x, y, z;
};

// Point with intensity
struct PointXYZI {
    float x, y, z;
    float intensity;
};

// Point with RGB color
struct PointXYZRGB {
    float x, y, z;
    uint32_t rgb;
};

// Point cloud container
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
```

### Python Point Cloud Processing

```python
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class PointCloudProcessor:
    def __init__(self):
        self.current_cloud = None
        
    def ros_to_numpy(self, ros_cloud):
        """Convert ROS PointCloud2 to numpy array"""
        points = []
        for point in pc2.read_points(ros_cloud, skip_nans=True):
            points.append([point[0], point[1], point[2]])
        
        return np.array(points, dtype=np.float32)
    
    def numpy_to_o3d(self, np_points):
        """Convert numpy array to Open3D point cloud"""
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np_points)
        return cloud
    
    def filter_by_distance(self, points, min_dist=0.1, max_dist=50.0):
        """Filter points by distance from origin"""
        distances = np.linalg.norm(points, axis=1)
        mask = (distances >= min_dist) & (distances <= max_dist)
        return points[mask]
    
    def filter_by_height(self, points, min_z=-2.0, max_z=3.0):
        """Filter points by height (Z coordinate)"""
        mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
        return points[mask]
    
    def downsample_voxel(self, cloud, voxel_size=0.1):
        """Downsample point cloud using voxel grid"""
        if isinstance(cloud, np.ndarray):
            cloud = self.numpy_to_o3d(cloud)
        
        downsampled = cloud.voxel_down_sample(voxel_size)
        return np.asarray(downsampled.points)
    
    def remove_outliers(self, cloud, nb_neighbors=20, std_ratio=2.0):
        """Remove statistical outliers"""
        if isinstance(cloud, np.ndarray):
            cloud = self.numpy_to_o3d(cloud)
        
        cleaned, _ = cloud.remove_statistical_outlier(nb_neighbors, std_ratio)
        return np.asarray(cleaned.points)
```

## Point Cloud Filtering and Preprocessing

### 1. Noise Removal

```python
def remove_noise(points, method='statistical'):
    """Remove noise from point cloud"""
    
    if method == 'statistical':
        # Statistical outlier removal
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        
        cleaned, _ = cloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0)
        return np.asarray(cleaned.points)
    
    elif method == 'radius':
        # Radius outlier removal
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        
        cleaned, _ = cloud.remove_radius_outlier(
            nb_points=16, radius=0.05)
        return np.asarray(cleaned.points)
    
    elif method == 'distance':
        # Simple distance-based filtering
        distances = np.linalg.norm(points, axis=1)
        valid_mask = (distances > 0.1) & (distances < 100.0)
        return points[valid_mask]

def temporal_filtering(point_history, alpha=0.7):
    """Apply temporal filtering to reduce noise"""
    if len(point_history) < 2:
        return point_history[-1]
    
    # Exponential moving average
    filtered = alpha * point_history[-1] + (1 - alpha) * point_history[-2]
    return filtered
```

### 2. Ground Plane Removal

```python
def remove_ground_plane(points, method='ransac'):
    """Remove ground plane from point cloud"""
    
    if method == 'ransac':
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        
        # RANSAC plane segmentation
        plane_model, inliers = cloud.segment_plane(
            distance_threshold=0.1,
            ransac_n=3,
            num_iterations=1000)
        
        # Remove ground points
        non_ground_cloud = cloud.select_by_index(inliers, invert=True)
        return np.asarray(non_ground_cloud.points)
    
    elif method == 'height_threshold':
        # Simple height-based ground removal
        ground_height = np.percentile(points[:, 2], 10)  # 10th percentile
        threshold = ground_height + 0.2  # 20cm above ground
        
        non_ground_mask = points[:, 2] > threshold
        return points[non_ground_mask]

def progressive_morphological_filter(points, cell_size=1.0, max_window_size=33):
    """Progressive morphological filter for ground removal"""
    # Implementation of Zhang et al. (2003) algorithm
    
    # Create height grid
    min_x, max_x = points[:, 0].min(), points[:, 0].max()
    min_y, max_y = points[:, 1].min(), points[:, 1].max()
    
    grid_width = int((max_x - min_x) / cell_size) + 1
    grid_height = int((max_y - min_y) / cell_size) + 1
    
    height_grid = np.full((grid_height, grid_width), np.inf)
    
    # Fill grid with minimum heights
    for point in points:
        x_idx = int((point[0] - min_x) / cell_size)
        y_idx = int((point[1] - min_y) / cell_size)
        
        if 0 <= x_idx < grid_width and 0 <= y_idx < grid_height:
            height_grid[y_idx, x_idx] = min(height_grid[y_idx, x_idx], point[2])
    
    # Apply morphological operations
    from scipy import ndimage
    
    ground_mask = np.zeros_like(height_grid, dtype=bool)
    
    for window_size in range(3, max_window_size + 1, 2):
        # Morphological opening
        kernel = np.ones((window_size, window_size))
        opened = ndimage.grey_opening(height_grid, structure=kernel)
        
        # Height difference threshold
        height_diff_threshold = 0.5 * (window_size - 1) * cell_size
        
        # Update ground mask
        height_diff = height_grid - opened
        ground_mask |= (height_diff < height_diff_threshold)
    
    # Apply mask to original points
    non_ground_points = []
    for point in points:
        x_idx = int((point[0] - min_x) / cell_size)
        y_idx = int((point[1] - min_y) / cell_size)
        
        if 0 <= x_idx < grid_width and 0 <= y_idx < grid_height:
            if not ground_mask[y_idx, x_idx]:
                non_ground_points.append(point)
    
    return np.array(non_ground_points)
```

## Object Detection in Point Clouds

### 1. Clustering-Based Detection

```python
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

def cluster_objects(points, eps=0.5, min_samples=10):
    """Detect objects using DBSCAN clustering"""
    
    # Apply DBSCAN clustering
    clustering = DBSCAN(eps=eps, min_samples=min_samples)
    cluster_labels = clustering.fit_predict(points[:, :3])  # Use XYZ only
    
    # Extract clusters
    clusters = []
    unique_labels = set(cluster_labels)
    
    for label in unique_labels:
        if label == -1:  # Noise points
            continue
            
        cluster_mask = cluster_labels == label
        cluster_points = points[cluster_mask]
        
        # Calculate cluster properties
        centroid = np.mean(cluster_points, axis=0)
        min_bounds = np.min(cluster_points, axis=0)
        max_bounds = np.max(cluster_points, axis=0)
        size = max_bounds - min_bounds
        
        clusters.append({
            'label': label,
            'points': cluster_points,
            'centroid': centroid,
            'min_bounds': min_bounds,
            'max_bounds': max_bounds,
            'size': size,
            'num_points': len(cluster_points)
        })
    
    return clusters

def euclidean_clustering(points, tolerance=0.5, min_size=10, max_size=10000):
    """Euclidean clustering for object segmentation"""
    
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    
    # Euclidean clustering
    labels = np.array(cloud.cluster_dbscan(
        eps=tolerance, min_points=min_size, print_progress=False))
    
    # Filter clusters by size
    clusters = []
    unique_labels = set(labels)
    
    for label in unique_labels:
        if label == -1:  # Noise
            continue
            
        cluster_mask = labels == label
        cluster_points = points[cluster_mask]
        
        if min_size <= len(cluster_points) <= max_size:
            clusters.append({
                'label': label,
                'points': cluster_points,
                'centroid': np.mean(cluster_points, axis=0),
                'size': len(cluster_points)
            })
    
    return clusters
```

### 2. Geometric Shape Detection

```python
def detect_cylinders(points, radius_range=(0.05, 0.5), height_range=(0.1, 2.0)):
    """Detect cylindrical objects in point cloud"""
    
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    
    cylinders = []
    remaining_points = points.copy()
    
    for iteration in range(10):  # Maximum iterations
        if len(remaining_points) < 100:
            break
            
        # RANSAC cylinder detection
        cloud_temp = o3d.geometry.PointCloud()
        cloud_temp.points = o3d.utility.Vector3dVector(remaining_points)
        
        # Custom RANSAC for cylinder detection
        best_cylinder = None
        best_inliers = []
        max_inliers = 0
        
        for _ in range(1000):  # RANSAC iterations
            # Sample 3 points
            if len(remaining_points) < 3:
                break
                
            sample_indices = np.random.choice(len(remaining_points), 3, replace=False)
            sample_points = remaining_points[sample_indices]
            
            # Fit cylinder to sample points
            cylinder_params = fit_cylinder_to_points(sample_points)
            if cylinder_params is None:
                continue
            
            # Check if cylinder parameters are valid
            radius = cylinder_params['radius']
            height = cylinder_params['height']
            
            if not (radius_range[0] <= radius <= radius_range[1] and
                    height_range[0] <= height <= height_range[1]):
                continue
            
            # Count inliers
            inliers = find_cylinder_inliers(remaining_points, cylinder_params, threshold=0.05)
            
            if len(inliers) > max_inliers:
                max_inliers = len(inliers)
                best_cylinder = cylinder_params
                best_inliers = inliers
        
        # Add cylinder if enough inliers found
        if max_inliers > 50:
            cylinders.append({
                'params': best_cylinder,
                'inliers': best_inliers,
                'num_inliers': max_inliers
            })
            
            # Remove inlier points
            inlier_mask = np.ones(len(remaining_points), dtype=bool)
            inlier_mask[best_inliers] = False
            remaining_points = remaining_points[inlier_mask]
        else:
            break
    
    return cylinders

def detect_planes(points, distance_threshold=0.1, num_iterations=1000):
    """Detect planar surfaces in point cloud"""
    
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    
    planes = []
    remaining_cloud = cloud
    
    for iteration in range(5):  # Maximum number of planes
        if len(remaining_cloud.points) < 100:
            break
        
        # RANSAC plane segmentation
        plane_model, inliers = remaining_cloud.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=num_iterations)
        
        if len(inliers) < 100:  # Minimum points for a plane
            break
        
        # Extract plane points
        plane_cloud = remaining_cloud.select_by_index(inliers)
        plane_points = np.asarray(plane_cloud.points)
        
        # Calculate plane properties
        normal = plane_model[:3]
        d = plane_model[3]
        
        # Calculate plane bounds
        min_bounds = np.min(plane_points, axis=0)
        max_bounds = np.max(plane_points, axis=0)
        
        planes.append({
            'model': plane_model,
            'normal': normal,
            'd': d,
            'points': plane_points,
            'min_bounds': min_bounds,
            'max_bounds': max_bounds,
            'area': len(plane_points) * 0.01,  # Approximate area
            'num_points': len(plane_points)
        })
        
        # Remove plane points from remaining cloud
        remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)
    
    return planes
```

## SLAM with LIDAR

### 1. Scan Matching

```python
import numpy as np
from scipy.optimize import minimize

class ScanMatcher:
    def __init__(self):
        self.reference_scan = None
        
    def icp_2d(self, source_points, target_points, max_iterations=50, tolerance=1e-6):
        """2D Iterative Closest Point algorithm"""
        
        # Initialize transformation
        transformation = np.eye(3)
        
        for iteration in range(max_iterations):
            # Find closest points
            correspondences = self.find_correspondences(source_points, target_points)
            
            if len(correspondences) < 3:
                break
            
            # Estimate transformation
            delta_transform = self.estimate_transformation_2d(correspondences)
            
            # Apply transformation
            transformation = np.dot(delta_transform, transformation)
            
            # Transform source points
            source_points = self.apply_transformation_2d(source_points, delta_transform)
            
            # Check convergence
            translation_change = np.linalg.norm(delta_transform[:2, 2])
            rotation_change = np.abs(np.arctan2(delta_transform[1, 0], delta_transform[0, 0]))
            
            if translation_change < tolerance and rotation_change < tolerance:
                break
        
        return transformation, source_points
    
    def find_correspondences(self, source_points, target_points, max_distance=1.0):
        """Find point correspondences between scans"""
        from scipy.spatial import cKDTree
        
        # Build KD-tree for target points
        tree = cKDTree(target_points)
        
        correspondences = []
        for i, source_point in enumerate(source_points):
            # Find nearest neighbor
            distance, target_idx = tree.query(source_point)
            
            if distance < max_distance:
                correspondences.append({
                    'source_idx': i,
                    'target_idx': target_idx,
                    'source_point': source_point,
                    'target_point': target_points[target_idx],
                    'distance': distance
                })
        
        return correspondences
    
    def estimate_transformation_2d(self, correspondences):
        """Estimate 2D transformation from correspondences"""
        
        if len(correspondences) < 3:
            return np.eye(3)
        
        # Extract point pairs
        source_points = np.array([c['source_point'] for c in correspondences])
        target_points = np.array([c['target_point'] for c in correspondences])
        
        # Calculate centroids
        source_centroid = np.mean(source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)
        
        # Center the points
        source_centered = source_points - source_centroid
        target_centered = target_points - target_centroid
        
        # Calculate rotation using SVD
        H = np.dot(source_centered.T, target_centered)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        # Ensure proper rotation matrix
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        # Calculate translation
        t = target_centroid - np.dot(R, source_centroid)
        
        # Build transformation matrix
        transformation = np.eye(3)
        transformation[:2, :2] = R
        transformation[:2, 2] = t
        
        return transformation
    
    def apply_transformation_2d(self, points, transformation):
        """Apply 2D transformation to points"""
        # Convert to homogeneous coordinates
        homogeneous_points = np.hstack([points, np.ones((len(points), 1))])
        
        # Apply transformation
        transformed = np.dot(transformation, homogeneous_points.T).T
        
        # Convert back to 2D
        return transformed[:, :2]
```

### 2. Occupancy Grid Mapping

```python
class OccupancyGridMapper:
    def __init__(self, resolution=0.1, width=1000, height=1000):
        self.resolution = resolution  # meters per cell
        self.width = width
        self.height = height
        
        # Initialize grid (0.5 = unknown, 0 = free, 1 = occupied)
        self.grid = np.full((height, width), 0.5, dtype=np.float32)
        
        # Grid origin in world coordinates
        self.origin_x = -width * resolution / 2
        self.origin_y = -height * resolution / 2
        
        # Log-odds representation for Bayesian updates
        self.log_odds = np.zeros((height, width), dtype=np.float32)
        
        # Probabilities for updates
        self.prob_hit = 0.7
        self.prob_miss = 0.3
        
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices"""
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates"""
        x = grid_x * self.resolution + self.origin_x
        y = grid_y * self.resolution + self.origin_y
        return x, y
    
    def update_with_scan(self, robot_pose, scan_ranges, scan_angles):
        """Update occupancy grid with laser scan"""
        robot_x, robot_y, robot_theta = robot_pose
        
        for range_val, angle in zip(scan_ranges, scan_angles):
            if np.isinf(range_val) or np.isnan(range_val):
                continue
            
            # Calculate end point of laser beam
            beam_angle = robot_theta + angle
            end_x = robot_x + range_val * np.cos(beam_angle)
            end_y = robot_y + range_val * np.sin(beam_angle)
            
            # Trace ray from robot to end point
            self.trace_ray(robot_x, robot_y, end_x, end_y, range_val)
    
    def trace_ray(self, x0, y0, x1, y1, max_range):
        """Trace ray and update grid cells"""
        # Bresenham's line algorithm in grid coordinates
        grid_x0, grid_y0 = self.world_to_grid(x0, y0)
        grid_x1, grid_y1 = self.world_to_grid(x1, y1)
        
        # Get all cells along the ray
        ray_cells = self.bresenham_line(grid_x0, grid_y0, grid_x1, grid_y1)
        
        for i, (gx, gy) in enumerate(ray_cells):
            if not self.is_valid_cell(gx, gy):
                continue
            
            # Calculate distance from robot
            world_x, world_y = self.grid_to_world(gx, gy)
            distance = np.sqrt((world_x - x0)**2 + (world_y - y0)**2)
            
            if i == len(ray_cells) - 1 and distance <= max_range:
                # Last cell - obstacle detected
                self.update_cell(gx, gy, self.prob_hit)
            else:
                # Intermediate cell - free space
                self.update_cell(gx, gy, self.prob_miss)
    
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm"""
        points = []
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            
            if e2 > -dy:
                err -= dy
                x += sx
            
            if e2 < dx:
                err += dx
                y += sy
        
        return points
    
    def update_cell(self, grid_x, grid_y, probability):
        """Update single cell using log-odds"""
        if not self.is_valid_cell(grid_x, grid_y):
            return
        
        # Convert probability to log-odds
        log_odds_update = np.log(probability / (1 - probability))
        
        # Update log-odds
        self.log_odds[grid_y, grid_x] += log_odds_update
        
        # Clamp log-odds to prevent overflow
        self.log_odds[grid_y, grid_x] = np.clip(self.log_odds[grid_y, grid_x], -10, 10)
        
        # Convert back to probability
        self.grid[grid_y, grid_x] = 1 / (1 + np.exp(-self.log_odds[grid_y, grid_x]))
    
    def is_valid_cell(self, grid_x, grid_y):
        """Check if grid cell is within bounds"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def get_occupancy_grid(self):
        """Get current occupancy grid"""
        return self.grid.copy()
```

## ROS 2 Integration

### LIDAR Node Implementation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class LidarProcessingNode(Node):
    def __init__(self):
        super().__init__('lidar_processing_node')
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Publishers
        self.filtered_scan_pub = self.create_publisher(
            LaserScan, '/scan_filtered', 10)
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, '/pointcloud', 10)
        
        # Point cloud processor
        self.processor = PointCloudProcessor()
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('LIDAR Processing Node initialized')
    
    def scan_callback(self, msg):
        """Process incoming laser scan"""
        try:
            # Convert scan to point cloud
            points = self.scan_to_pointcloud(msg)
            
            # Filter points
            filtered_points = self.processor.filter_by_distance(points, 0.1, 30.0)
            filtered_points = self.processor.remove_outliers(filtered_points)
            
            # Convert back to laser scan
            filtered_scan = self.pointcloud_to_scan(filtered_points, msg)
            
            # Publish filtered scan
            self.filtered_scan_pub.publish(filtered_scan)
            
            # Publish point cloud
            pointcloud_msg = self.create_pointcloud_msg(filtered_points, msg.header)
            self.pointcloud_pub.publish(pointcloud_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing scan: {str(e)}')
    
    def scan_to_pointcloud(self, scan_msg):
        """Convert LaserScan to point cloud"""
        points = []
        
        for i, range_val in enumerate(scan_msg.ranges):
            if np.isinf(range_val) or np.isnan(range_val):
                continue
            
            if range_val < scan_msg.range_min or range_val > scan_msg.range_max:
                continue
            
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            z = 0.0
            
            points.append([x, y, z])
        
        return np.array(points, dtype=np.float32)
    
    def pointcloud_to_scan(self, points, original_scan):
        """Convert point cloud back to LaserScan"""
        # Create new scan message
        scan = LaserScan()
        scan.header = original_scan.header
        scan.angle_min = original_scan.angle_min
        scan.angle_max = original_scan.angle_max
        scan.angle_increment = original_scan.angle_increment
        scan.time_increment = original_scan.time_increment
        scan.scan_time = original_scan.scan_time
        scan.range_min = original_scan.range_min
        scan.range_max = original_scan.range_max
        
        # Initialize ranges array
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [float('inf')] * num_readings
        
        # Fill ranges from point cloud
        for point in points:
            x, y = point[0], point[1]
            
            # Calculate range and angle
            range_val = np.sqrt(x*x + y*y)
            angle = np.arctan2(y, x)
            
            # Find corresponding index
            if scan.angle_min <= angle <= scan.angle_max:
                index = int((angle - scan.angle_min) / scan.angle_increment)
                if 0 <= index < len(scan.ranges):
                    # Keep minimum range for each angle
                    scan.ranges[index] = min(scan.ranges[index], range_val)
        
        return scan
    
    def create_pointcloud_msg(self, points, header):
        """Create PointCloud2 message from numpy array"""
        # Create point cloud message
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(points)
        pointcloud_msg.is_dense = True
        
        # Convert points to PointCloud2 format
        pointcloud_msg = pc2.create_cloud_xyz32(header, points.tolist())
        
        return pointcloud_msg

def main():
    rclpy.init()
    lidar_node = LidarProcessingNode()
    
    try:
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization

### 1. Efficient Point Cloud Processing

```python
import numba
from numba import jit

@jit(nopython=True)
def fast_distance_filter(points, min_dist, max_dist):
    """Fast distance filtering using Numba JIT compilation"""
    filtered_points = []
    
    for i in range(points.shape[0]):
        x, y, z = points[i, 0], points[i, 1], points[i, 2]
        dist = np.sqrt(x*x + y*y + z*z)
        
        if min_dist <= dist <= max_dist:
            filtered_points.append([x, y, z])
    
    return np.array(filtered_points)

@jit(nopython=True)
def fast_voxel_downsample(points, voxel_size):
    """Fast voxel downsampling"""
    if len(points) == 0:
        return points
    
    # Find bounds
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    
    # Calculate grid dimensions
    grid_size = ((max_bounds - min_bounds) / voxel_size).astype(np.int32) + 1
    
    # Create voxel grid
    voxel_dict = {}
    
    for i in range(points.shape[0]):
        point = points[i]
        
        # Calculate voxel indices
        voxel_idx = ((point - min_bounds) / voxel_size).astype(np.int32)
        
        # Create voxel key
        key = (voxel_idx[0], voxel_idx[1], voxel_idx[2])
        
        if key not in voxel_dict:
            voxel_dict[key] = []
        
        voxel_dict[key].append(point)
    
    # Calculate centroids
    downsampled_points = []
    for voxel_points in voxel_dict.values():
        if len(voxel_points) > 0:
            centroid = np.mean(np.array(voxel_points), axis=0)
            downsampled_points.append(centroid)
    
    return np.array(downsampled_points)
```

### 2. Multi-threaded Processing

```python
import threading
import queue
from concurrent.futures import ThreadPoolExecutor

class ParallelPointCloudProcessor:
    def __init__(self, num_workers=4):
        self.num_workers = num_workers
        self.executor = ThreadPoolExecutor(max_workers=num_workers)
        
    def process_scan_parallel(self, scan_data):
        """Process laser scan using parallel processing"""
        
        # Split scan into chunks
        chunk_size = len(scan_data.ranges) // self.num_workers
        chunks = []
        
        for i in range(self.num_workers):
            start_idx = i * chunk_size
            end_idx = start_idx + chunk_size if i < self.num_workers - 1 else len(scan_data.ranges)
            
            chunk = {
                'ranges': scan_data.ranges[start_idx:end_idx],
                'start_angle': scan_data.angle_min + start_idx * scan_data.angle_increment,
                'angle_increment': scan_data.angle_increment,
                'start_idx': start_idx
            }
            chunks.append(chunk)
        
        # Process chunks in parallel
        futures = []
        for chunk in chunks:
            future = self.executor.submit(self.process_scan_chunk, chunk)
            futures.append(future)
        
        # Collect results
        all_points = []
        for future in futures:
            chunk_points = future.result()
            all_points.extend(chunk_points)
        
        return np.array(all_points)
    
    def process_scan_chunk(self, chunk):
        """Process a chunk of scan data"""
        points = []
        
        for i, range_val in enumerate(chunk['ranges']):
            if np.isinf(range_val) or np.isnan(range_val):
                continue
            
            angle = chunk['start_angle'] + i * chunk['angle_increment']
            
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            z = 0.0
            
            points.append([x, y, z])
        
        return points
```

## Key Takeaways

- LIDAR provides precise distance measurements for robot perception and navigation
- 2D LIDAR is suitable for mobile robot navigation, while 3D LIDAR enables detailed environment mapping
- Point cloud processing involves filtering, segmentation, and object detection
- Ground plane removal is essential for obstacle detection in mobile robots
- Clustering algorithms like DBSCAN effectively segment objects in point clouds
- SLAM algorithms use scan matching and occupancy grid mapping for navigation
- ROS 2 integration enables seamless LIDAR data processing in robot systems
- Performance optimization through JIT compilation and parallel processing is crucial for real-time applications
- Proper calibration and filtering improve LIDAR data quality and reliability

---

**Next:** [Lecture 3: Sensor Fusion and State Estimation](./lecture-3.md)