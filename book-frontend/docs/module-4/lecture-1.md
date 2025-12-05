---
sidebar_position: 1
---

# Lecture 1: Computer Vision for Robots

## Introduction to Robot Vision

Computer vision is the **eyes** of a robot - it enables robots to perceive, understand, and interact with their visual environment. Unlike human vision, robot vision can be enhanced with multiple cameras, different light spectrums, and AI-powered analysis.

## Why Computer Vision Matters for Robots

### Human vs Robot Vision

| Aspect | Human Vision | Robot Vision |
|--------|--------------|--------------|
| **Spectrum** | Visible light (400-700nm) | Visible + IR + UV + Depth |
| **Processing** | Biological neural networks | Digital algorithms + AI |
| **Accuracy** | Good pattern recognition | Precise measurements |
| **Speed** | ~24 FPS perception | Up to 1000+ FPS |
| **Memory** | Selective, contextual | Perfect recall, searchable |
| **Fatigue** | Gets tired, attention varies | Consistent performance |

### Robot Vision Applications

#### 1. Navigation and Mapping
- **Obstacle detection**: Identify and avoid obstacles
- **Path planning**: Find optimal routes
- **SLAM**: Simultaneous Localization and Mapping
- **Visual odometry**: Track movement using visual features

#### 2. Object Recognition and Manipulation
- **Object detection**: Find specific objects in scenes
- **Pose estimation**: Determine object position and orientation
- **Grasping**: Guide robot hands to pick up objects
- **Quality inspection**: Detect defects in manufacturing

#### 3. Human-Robot Interaction
- **Face recognition**: Identify specific people
- **Gesture recognition**: Understand human gestures
- **Emotion detection**: Recognize facial expressions
- **Safety monitoring**: Detect humans in robot workspace

## Camera Systems for Robots

### Types of Cameras

#### 1. RGB Cameras (Color)
```python
# Basic RGB camera configuration
camera_config = {
    'resolution': (1920, 1080),  # Full HD
    'fps': 30,                   # Frames per second
    'format': 'RGB8',           # Color format
    'field_of_view': 60,        # Degrees
    'auto_exposure': True,
    'auto_white_balance': True
}
```

**Advantages:**
- Rich color information
- Good for object recognition
- Human-interpretable images
- Widely supported

**Disadvantages:**
- No depth information
- Sensitive to lighting conditions
- Limited range information

#### 2. Depth Cameras
```python
# Depth camera configuration
depth_config = {
    'depth_resolution': (640, 480),
    'rgb_resolution': (1920, 1080),
    'depth_fps': 30,
    'rgb_fps': 30,
    'min_depth': 0.1,           # meters
    'max_depth': 10.0,          # meters
    'depth_accuracy': 0.001     # 1mm accuracy
}
```

**Popular Depth Cameras:**
- **Intel RealSense D435i**: RGB-D with IMU
- **Microsoft Kinect**: Gaming to robotics
- **Orbbec Astra**: Affordable RGB-D
- **ZED Camera**: Stereo depth camera

#### 3. Stereo Cameras
```python
# Stereo camera setup
stereo_config = {
    'baseline': 0.12,           # Distance between cameras (m)
    'focal_length': 3.6,        # mm
    'resolution': (1280, 720),
    'calibration_required': True,
    'disparity_range': (0, 64)
}
```

**Advantages:**
- Passive depth sensing
- Works in any lighting
- No interference between multiple units
- Good for outdoor use

**Disadvantages:**
- Requires calibration
- Computationally intensive
- Struggles with textureless surfaces

### Camera Calibration

Camera calibration determines the camera's intrinsic and extrinsic parameters.

#### Intrinsic Parameters
```python
import cv2
import numpy as np

# Camera matrix (intrinsic parameters)
camera_matrix = np.array([
    [fx,  0, cx],  # fx = focal length in x
    [ 0, fy, cy],  # fy = focal length in y
    [ 0,  0,  1]   # cx, cy = principal point
])

# Distortion coefficients
dist_coeffs = np.array([k1, k2, p1, p2, k3])  # Radial and tangential distortion
```

#### Calibration Process
```python
def calibrate_camera(calibration_images, chessboard_size):
    """
    Calibrate camera using chessboard pattern
    """
    # Prepare object points (3D points in real world space)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    
    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    
    for image_path in calibration_images:
        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        
        if ret:
            objpoints.append(objp)
            
            # Refine corner positions
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                      (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            imgpoints.append(corners2)
    
    # Calibrate camera
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)
    
    return camera_matrix, dist_coeffs

# Usage
chessboard_size = (9, 6)  # Internal corners
calibration_images = ['calib1.jpg', 'calib2.jpg', ...]  # Multiple images
camera_matrix, dist_coeffs = calibrate_camera(calibration_images, chessboard_size)
```

## Image Processing Fundamentals

### Basic Image Operations

#### 1. Image Filtering
```python
import cv2
import numpy as np

def preprocess_image(image):
    """Basic image preprocessing pipeline"""
    
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Histogram equalization for better contrast
    equalized = cv2.equalizeHist(blurred)
    
    # Edge detection
    edges = cv2.Canny(equalized, 50, 150)
    
    return {
        'original': image,
        'gray': gray,
        'blurred': blurred,
        'equalized': equalized,
        'edges': edges
    }

# Example usage
image = cv2.imread('robot_view.jpg')
processed = preprocess_image(image)
```

#### 2. Morphological Operations
```python
def morphological_operations(binary_image):
    """Apply morphological operations to clean up binary images"""
    
    # Define kernel
    kernel = np.ones((5, 5), np.uint8)
    
    # Erosion - removes small noise
    eroded = cv2.erode(binary_image, kernel, iterations=1)
    
    # Dilation - fills small holes
    dilated = cv2.dilate(binary_image, kernel, iterations=1)
    
    # Opening - erosion followed by dilation (removes noise)
    opened = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
    
    # Closing - dilation followed by erosion (fills holes)
    closed = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel)
    
    return {
        'eroded': eroded,
        'dilated': dilated,
        'opened': opened,
        'closed': closed
    }
```

### Feature Detection

#### 1. Corner Detection
```python
def detect_corners(image):
    """Detect corners using different methods"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Harris corner detection
    harris_corners = cv2.cornerHarris(gray, 2, 3, 0.04)
    
    # Good Features to Track (Shi-Tomasi)
    corners_shi_tomasi = cv2.goodFeaturesToTrack(
        gray, 
        maxCorners=100,
        qualityLevel=0.01,
        minDistance=10,
        blockSize=3
    )
    
    # FAST corner detection
    fast = cv2.FastFeatureDetector_create()
    keypoints_fast = fast.detect(gray, None)
    
    return {
        'harris': harris_corners,
        'shi_tomasi': corners_shi_tomasi,
        'fast': keypoints_fast
    }
```

#### 2. Feature Descriptors
```python
def extract_features(image):
    """Extract features using different descriptors"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # SIFT (Scale-Invariant Feature Transform)
    sift = cv2.SIFT_create()
    keypoints_sift, descriptors_sift = sift.detectAndCompute(gray, None)
    
    # ORB (Oriented FAST and Rotated BRIEF)
    orb = cv2.ORB_create()
    keypoints_orb, descriptors_orb = orb.detectAndCompute(gray, None)
    
    # SURF (Speeded-Up Robust Features) - if available
    try:
        surf = cv2.xfeatures2d.SURF_create(400)
        keypoints_surf, descriptors_surf = surf.detectAndCompute(gray, None)
    except:
        keypoints_surf, descriptors_surf = None, None
    
    return {
        'sift': (keypoints_sift, descriptors_sift),
        'orb': (keypoints_orb, descriptors_orb),
        'surf': (keypoints_surf, descriptors_surf)
    }

def match_features(desc1, desc2, method='bf'):
    """Match features between two images"""
    if method == 'bf':
        # Brute Force matcher
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(desc1, desc2, k=2)
        
        # Apply Lowe's ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)
                
    elif method == 'flann':
        # FLANN matcher
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(desc1, desc2, k=2)
        
        # Apply Lowe's ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)
    
    return good_matches
```

## Object Detection

### Traditional Computer Vision Approaches

#### 1. Template Matching
```python
def template_matching(image, template):
    """Find template in image using template matching"""
    
    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    
    # Template matching
    result = cv2.matchTemplate(gray_image, gray_template, cv2.TM_CCOEFF_NORMED)
    
    # Find best match
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    
    # Get template dimensions
    h, w = gray_template.shape
    
    # Draw rectangle around best match
    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)
    
    return {
        'confidence': max_val,
        'location': top_left,
        'bounding_box': (top_left[0], top_left[1], w, h)
    }
```

#### 2. Contour-Based Detection
```python
def detect_objects_by_contour(image, min_area=1000):
    """Detect objects using contour analysis"""
    
    # Convert to grayscale and threshold
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    
    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detected_objects = []
    
    for contour in contours:
        # Filter by area
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
            
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        
        # Calculate shape properties
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        
        # Approximate contour to polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        detected_objects.append({
            'contour': contour,
            'bounding_box': (x, y, w, h),
            'area': area,
            'perimeter': perimeter,
            'circularity': circularity,
            'vertices': len(approx),
            'center': (x + w//2, y + h//2)
        })
    
    return detected_objects
```

### Deep Learning Approaches

#### 1. YOLO (You Only Look Once)
```python
import cv2
import numpy as np

class YOLODetector:
    def __init__(self, weights_path, config_path, classes_path):
        """Initialize YOLO detector"""
        self.net = cv2.dnn.readNet(weights_path, config_path)
        
        # Load class names
        with open(classes_path, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]
        
        # Get output layer names
        layer_names = self.net.getLayerNames()
        self.output_layers = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        
        # Generate colors for each class
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
    
    def detect(self, image, confidence_threshold=0.5, nms_threshold=0.4):
        """Detect objects in image"""
        height, width, channels = image.shape
        
        # Prepare input blob
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        
        # Run inference
        outputs = self.net.forward(self.output_layers)
        
        # Process outputs
        boxes = []
        confidences = []
        class_ids = []
        
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                if confidence > confidence_threshold:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    
                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        # Apply Non-Maximum Suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, confidence_threshold, nms_threshold)
        
        detections = []
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                class_name = self.classes[class_ids[i]]
                confidence = confidences[i]
                
                detections.append({
                    'class': class_name,
                    'confidence': confidence,
                    'bounding_box': (x, y, w, h),
                    'center': (x + w//2, y + h//2)
                })
        
        return detections
    
    def draw_detections(self, image, detections):
        """Draw detection results on image"""
        result_image = image.copy()
        
        for detection in detections:
            x, y, w, h = detection['bounding_box']
            class_name = detection['class']
            confidence = detection['confidence']
            
            # Get class color
            class_id = self.classes.index(class_name)
            color = self.colors[class_id]
            
            # Draw bounding box
            cv2.rectangle(result_image, (x, y), (x + w, y + h), color, 2)
            
            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(result_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return result_image

# Usage example
detector = YOLODetector('yolov3.weights', 'yolov3.cfg', 'coco.names')
image = cv2.imread('robot_scene.jpg')
detections = detector.detect(image)
result_image = detector.draw_detections(image, detections)
```

## ROS 2 Integration

### Camera Node Implementation
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)  # Use first camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Timer for publishing images
        self.timer = self.create_timer(1.0/30.0, self.publish_image)  # 30 FPS
        
        # Camera calibration parameters (from calibration)
        self.camera_matrix = np.array([
            [525.0, 0.0, 320.0],
            [0.0, 525.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([0.1, -0.2, 0.0, 0.0, 0.0])
    
    def publish_image(self):
        """Capture and publish camera image"""
        ret, frame = self.cap.read()
        
        if ret:
            # Undistort image
            undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            
            # Convert to ROS message
            image_msg = self.bridge.cv2_to_imgmsg(undistorted, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_link'
            
            # Publish image
            self.image_pub.publish(image_msg)
            
            # Publish camera info
            camera_info_msg = self.create_camera_info_msg()
            self.camera_info_pub.publish(camera_info_msg)
    
    def create_camera_info_msg(self):
        """Create camera info message"""
        camera_info = CameraInfo()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = 'camera_link'
        
        camera_info.width = 640
        camera_info.height = 480
        
        # Camera matrix (K)
        camera_info.k = self.camera_matrix.flatten().tolist()
        
        # Distortion coefficients (D)
        camera_info.d = self.dist_coeffs.tolist()
        
        # Rectification matrix (R) - identity for monocular
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix (P)
        camera_info.p = [
            self.camera_matrix[0, 0], 0.0, self.camera_matrix[0, 2], 0.0,
            0.0, self.camera_matrix[1, 1], self.camera_matrix[1, 2], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return camera_info
    
    def destroy_node(self):
        """Clean up resources"""
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Object Detection Node
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Subscriber for camera images
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.debug_image_pub = self.create_publisher(Image, '/detection_debug', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Initialize YOLO detector
        self.detector = YOLODetector('yolov3.weights', 'yolov3.cfg', 'coco.names')
        
        self.get_logger().info('Object Detection Node initialized')
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect objects
            detections = self.detector.detect(cv_image)
            
            # Create detection message
            detection_array = Detection2DArray()
            detection_array.header = Header()
            detection_array.header.stamp = self.get_clock().now().to_msg()
            detection_array.header.frame_id = msg.header.frame_id
            
            for detection in detections:
                # Create Detection2D message
                det_msg = Detection2D()
                
                # Bounding box
                x, y, w, h = detection['bounding_box']
                det_msg.bbox.center.x = float(x + w/2)
                det_msg.bbox.center.y = float(y + h/2)
                det_msg.bbox.size_x = float(w)
                det_msg.bbox.size_y = float(h)
                
                # Object hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = detection['class']
                hypothesis.score = detection['confidence']
                det_msg.results.append(hypothesis)
                
                detection_array.detections.append(det_msg)
            
            # Publish detections
            self.detection_pub.publish(detection_array)
            
            # Publish debug image
            debug_image = self.detector.draw_detections(cv_image, detections)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main():
    rclpy.init()
    detection_node = ObjectDetectionNode()
    
    try:
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Applications

### 1. Visual Navigation
```python
class VisualNavigationSystem:
    def __init__(self):
        self.feature_detector = cv2.ORB_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
    def visual_odometry(self, prev_frame, curr_frame, camera_matrix):
        """Estimate robot motion using visual odometry"""
        
        # Detect features in both frames
        kp1, desc1 = self.feature_detector.detectAndCompute(prev_frame, None)
        kp2, desc2 = self.feature_detector.detectAndCompute(curr_frame, None)
        
        if desc1 is None or desc2 is None:
            return None
        
        # Match features
        matches = self.matcher.match(desc1, desc2)
        matches = sorted(matches, key=lambda x: x.distance)
        
        if len(matches) < 10:
            return None
        
        # Extract matched points
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        
        # Find essential matrix
        essential_matrix, mask = cv2.findEssentialMat(
            pts1, pts2, camera_matrix, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        
        # Recover pose
        _, R, t, mask = cv2.recoverPose(essential_matrix, pts1, pts2, camera_matrix)
        
        return {
            'rotation': R,
            'translation': t,
            'num_matches': len(matches)
        }
```

### 2. Object Tracking
```python
class ObjectTracker:
    def __init__(self):
        self.trackers = {}
        self.next_id = 0
        
    def update_tracks(self, detections, frame):
        """Update object tracks with new detections"""
        
        # Update existing trackers
        active_trackers = {}
        for track_id, tracker in self.trackers.items():
            success, bbox = tracker.update(frame)
            if success:
                active_trackers[track_id] = {
                    'tracker': tracker,
                    'bbox': bbox,
                    'updated': True
                }
        
        # Match detections to existing tracks
        matched_detections = set()
        for track_id, track_info in active_trackers.items():
            best_match = None
            best_iou = 0.3  # Minimum IoU threshold
            
            for i, detection in enumerate(detections):
                if i in matched_detections:
                    continue
                    
                iou = self.calculate_iou(track_info['bbox'], detection['bounding_box'])
                if iou > best_iou:
                    best_iou = iou
                    best_match = i
            
            if best_match is not None:
                matched_detections.add(best_match)
                track_info['detection'] = detections[best_match]
        
        # Create new tracks for unmatched detections
        for i, detection in enumerate(detections):
            if i not in matched_detections:
                tracker = cv2.TrackerCSRT_create()
                bbox = detection['bounding_box']
                tracker.init(frame, bbox)
                
                active_trackers[self.next_id] = {
                    'tracker': tracker,
                    'bbox': bbox,
                    'detection': detection,
                    'updated': True
                }
                self.next_id += 1
        
        self.trackers = active_trackers
        return self.trackers
    
    def calculate_iou(self, bbox1, bbox2):
        """Calculate Intersection over Union of two bounding boxes"""
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        
        # Calculate intersection
        xi1 = max(x1, x2)
        yi1 = max(y1, y2)
        xi2 = min(x1 + w1, x2 + w2)
        yi2 = min(y1 + h1, y2 + h2)
        
        if xi2 <= xi1 or yi2 <= yi1:
            return 0
        
        intersection = (xi2 - xi1) * (yi2 - yi1)
        union = w1 * h1 + w2 * h2 - intersection
        
        return intersection / union if union > 0 else 0
```

## Performance Optimization

### 1. Image Processing Optimization
```python
def optimize_image_processing(image, target_size=(320, 240)):
    """Optimize image for faster processing"""
    
    # Resize image to reduce computation
    height, width = image.shape[:2]
    if width > target_size[0] or height > target_size[1]:
        image = cv2.resize(image, target_size)
    
    # Convert to grayscale if color not needed
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    
    return blurred

def roi_processing(image, roi_regions):
    """Process only regions of interest"""
    results = []
    
    for roi in roi_regions:
        x, y, w, h = roi
        roi_image = image[y:y+h, x:x+w]
        
        # Process ROI
        processed_roi = process_roi(roi_image)
        results.append({
            'roi': roi,
            'result': processed_roi
        })
    
    return results
```

### 2. Multi-threading for Real-time Processing
```python
import threading
import queue
from concurrent.futures import ThreadPoolExecutor

class RealTimeVisionProcessor:
    def __init__(self, max_workers=4):
        self.frame_queue = queue.Queue(maxsize=10)
        self.result_queue = queue.Queue()
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.running = False
        
    def start_processing(self):
        """Start real-time processing"""
        self.running = True
        
        # Start frame processing thread
        processing_thread = threading.Thread(target=self._process_frames)
        processing_thread.start()
        
    def add_frame(self, frame):
        """Add frame to processing queue"""
        if not self.frame_queue.full():
            self.frame_queue.put(frame)
        else:
            # Drop oldest frame if queue is full
            try:
                self.frame_queue.get_nowait()
                self.frame_queue.put(frame)
            except queue.Empty:
                pass
    
    def _process_frames(self):
        """Process frames in separate thread"""
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=1.0)
                
                # Submit frame for processing
                future = self.executor.submit(self._process_single_frame, frame)
                
                # Store future for result retrieval
                self.result_queue.put(future)
                
            except queue.Empty:
                continue
    
    def _process_single_frame(self, frame):
        """Process a single frame"""
        # Implement your vision processing here
        processed_result = {
            'timestamp': time.time(),
            'detections': [],  # Your detection results
            'features': []     # Your feature extraction results
        }
        
        return processed_result
    
    def get_latest_result(self):
        """Get latest processing result"""
        latest_result = None
        
        # Get all available results, keep only the latest
        while not self.result_queue.empty():
            try:
                future = self.result_queue.get_nowait()
                if future.done():
                    latest_result = future.result()
            except queue.Empty:
                break
        
        return latest_result
```

## Key Takeaways

- Computer vision enables robots to perceive and understand their visual environment
- Camera calibration is essential for accurate measurements and 3D reconstruction
- Feature detection and matching form the basis of many vision algorithms
- Deep learning approaches like YOLO provide robust object detection capabilities
- ROS 2 integration enables seamless vision processing in robot systems
- Performance optimization is crucial for real-time robot vision applications
- Visual navigation and object tracking are key applications in robotics
- Proper image preprocessing and ROI processing improve efficiency
- Multi-threading enables real-time processing of video streams

---

**Next:** [Lecture 2: LIDAR and Point Cloud Processing](./lecture-2.md)