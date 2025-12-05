---
sidebar_position: 2
---

# Lecture 2: Gesture and Facial Recognition

## Introduction to Visual Human-Robot Interaction

**Gesture and Facial Recognition** enables robots to understand non-verbal human communication, making interactions more natural and intuitive. This capability allows robots to interpret human emotions, intentions, and commands through visual cues.

## Why Visual Recognition Matters

### Non-Verbal Communication Statistics
- **55%** of human communication is body language
- **38%** is tone of voice
- **7%** is actual words

### Applications in Robotics
- **Gesture commands**: Point-and-click robot control
- **Emotion recognition**: Adaptive robot behavior
- **Safety monitoring**: Detecting human distress or danger
- **Social interaction**: Natural human-like responses

## Gesture Recognition

### Types of Gestures

#### 1. Static Gestures (Hand Poses)
```python
import cv2
import mediapipe as mp
import numpy as np
import math

class HandGestureRecognizer:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Define gesture patterns
        self.gesture_patterns = {
            'thumbs_up': self.is_thumbs_up,
            'peace_sign': self.is_peace_sign,
            'pointing': self.is_pointing,
            'open_palm': self.is_open_palm,
            'fist': self.is_fist,
            'ok_sign': self.is_ok_sign
        }
    
    def recognize_gesture(self, image):
        """Recognize hand gestures in image"""
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_image)
        
        gestures = []
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Extract landmark coordinates
                landmarks = self.extract_landmarks(hand_landmarks, image.shape)
                
                # Recognize gesture
                gesture = self.classify_gesture(landmarks)
                
                if gesture:
                    gestures.append({
                        'gesture': gesture,
                        'landmarks': landmarks,
                        'confidence': self.calculate_confidence(gesture, landmarks)
                    })
        
        return gestures
    
    def extract_landmarks(self, hand_landmarks, image_shape):
        """Extract normalized landmark coordinates"""
        h, w = image_shape[:2]
        landmarks = []
        
        for landmark in hand_landmarks.landmark:
            x = int(landmark.x * w)
            y = int(landmark.y * h)
            landmarks.append([x, y])
        
        return np.array(landmarks)
    
    def classify_gesture(self, landmarks):
        """Classify gesture based on landmark positions"""
        for gesture_name, gesture_func in self.gesture_patterns.items():
            if gesture_func(landmarks):
                return gesture_name
        
        return None
    
    def is_thumbs_up(self, landmarks):
        """Check if gesture is thumbs up"""
        # Thumb tip (4) should be above thumb IP (3)
        # Other fingers should be folded
        
        thumb_tip = landmarks[4]
        thumb_ip = landmarks[3]
        
        # Check if thumb is extended upward
        if thumb_tip[1] > thumb_ip[1]:  # Y increases downward
            return False
        
        # Check if other fingers are folded
        finger_tips = [8, 12, 16, 20]  # Index, middle, ring, pinky tips
        finger_pips = [6, 10, 14, 18]  # Corresponding PIP joints
        
        folded_fingers = 0
        for tip, pip in zip(finger_tips, finger_pips):
            if landmarks[tip][1] > landmarks[pip][1]:  # Tip below PIP = folded
                folded_fingers += 1
        
        return folded_fingers >= 3
    
    def is_peace_sign(self, landmarks):
        """Check if gesture is peace sign (V)"""
        # Index and middle fingers extended, others folded
        
        index_tip = landmarks[8]
        index_pip = landmarks[6]
        middle_tip = landmarks[12]
        middle_pip = landmarks[10]
        
        # Check if index and middle fingers are extended
        index_extended = index_tip[1] < index_pip[1]
        middle_extended = middle_tip[1] < middle_pip[1]
        
        if not (index_extended and middle_extended):
            return False
        
        # Check if ring and pinky are folded
        ring_tip = landmarks[16]
        ring_pip = landmarks[14]
        pinky_tip = landmarks[20]
        pinky_pip = landmarks[18]
        
        ring_folded = ring_tip[1] > ring_pip[1]
        pinky_folded = pinky_tip[1] > pinky_pip[1]
        
        return ring_folded and pinky_folded
    
    def is_pointing(self, landmarks):
        """Check if gesture is pointing"""
        # Only index finger extended
        
        index_tip = landmarks[8]
        index_pip = landmarks[6]
        
        # Index finger should be extended
        if index_tip[1] > index_pip[1]:
            return False
        
        # Other fingers should be folded
        other_tips = [12, 16, 20]  # Middle, ring, pinky
        other_pips = [10, 14, 18]
        
        folded_count = 0
        for tip, pip in zip(other_tips, other_pips):
            if landmarks[tip][1] > landmarks[pip][1]:
                folded_count += 1
        
        return folded_count >= 2
    
    def is_open_palm(self, landmarks):
        """Check if gesture is open palm"""
        # All fingers extended
        
        finger_tips = [4, 8, 12, 16, 20]  # Thumb, index, middle, ring, pinky
        finger_pips = [3, 6, 10, 14, 18]
        
        extended_count = 0
        for tip, pip in zip(finger_tips, finger_pips):
            if landmarks[tip][1] < landmarks[pip][1]:
                extended_count += 1
        
        return extended_count >= 4
    
    def is_fist(self, landmarks):
        """Check if gesture is fist"""
        # All fingers folded
        
        finger_tips = [8, 12, 16, 20]  # Index, middle, ring, pinky
        finger_pips = [6, 10, 14, 18]
        
        folded_count = 0
        for tip, pip in zip(finger_tips, finger_pips):
            if landmarks[tip][1] > landmarks[pip][1]:
                folded_count += 1
        
        return folded_count >= 3
    
    def is_ok_sign(self, landmarks):
        """Check if gesture is OK sign"""
        # Thumb and index finger form circle, others extended
        
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        
        # Calculate distance between thumb and index finger tips
        distance = math.sqrt((thumb_tip[0] - index_tip[0])**2 + 
                           (thumb_tip[1] - index_tip[1])**2)
        
        # If tips are close, might be OK sign
        if distance > 50:  # Adjust threshold as needed
            return False
        
        # Check if other fingers are extended
        other_tips = [12, 16, 20]
        other_pips = [10, 14, 18]
        
        extended_count = 0
        for tip, pip in zip(other_tips, other_pips):
            if landmarks[tip][1] < landmarks[pip][1]:
                extended_count += 1
        
        return extended_count >= 2
    
    def calculate_confidence(self, gesture, landmarks):
        """Calculate confidence score for recognized gesture"""
        # Simple confidence based on gesture clarity
        # In practice, this could be more sophisticated
        
        base_confidence = 0.8
        
        # Add confidence based on hand stability
        # (This would require tracking over multiple frames)
        
        return base_confidence
    
    def draw_landmarks(self, image, gestures):
        """Draw hand landmarks and gesture labels on image"""
        annotated_image = image.copy()
        
        for gesture_info in gestures:
            landmarks = gesture_info['landmarks']
            gesture_name = gesture_info['gesture']
            confidence = gesture_info['confidence']
            
            # Draw landmarks
            for landmark in landmarks:
                cv2.circle(annotated_image, tuple(landmark), 5, (0, 255, 0), -1)
            
            # Draw gesture label
            if len(landmarks) > 0:
                label_pos = (landmarks[0][0], landmarks[0][1] - 20)
                label = f"{gesture_name}: {confidence:.2f}"
                cv2.putText(annotated_image, label, label_pos, 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        return annotated_image

# Example usage
def demo_gesture_recognition():
    """Demo gesture recognition with webcam"""
    
    recognizer = HandGestureRecognizer()
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Recognize gestures
        gestures = recognizer.recognize_gesture(frame)
        
        # Draw results
        annotated_frame = recognizer.draw_landmarks(frame, gestures)
        
        # Display
        cv2.imshow('Gesture Recognition', annotated_frame)
        
        # Print recognized gestures
        for gesture_info in gestures:
            print(f"Gesture: {gesture_info['gesture']}, "
                  f"Confidence: {gesture_info['confidence']:.2f}")
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
```

#### 2. Dynamic Gestures (Motion-based)

```python
import numpy as np
from collections import deque
import time

class DynamicGestureRecognizer:
    def __init__(self, window_size=30):
        self.window_size = window_size
        self.landmark_history = deque(maxlen=window_size)
        self.gesture_templates = self.load_gesture_templates()
        
    def load_gesture_templates(self):
        """Load pre-defined gesture templates"""
        return {
            'wave': self.create_wave_template(),
            'swipe_left': self.create_swipe_left_template(),
            'swipe_right': self.create_swipe_right_template(),
            'circle': self.create_circle_template(),
            'come_here': self.create_come_here_template()
        }
    
    def add_landmarks(self, landmarks):
        """Add new landmark data to history"""
        if landmarks is not None and len(landmarks) > 0:
            # Use index finger tip for dynamic gestures
            index_tip = landmarks[8] if len(landmarks) > 8 else landmarks[0]
            
            self.landmark_history.append({
                'position': index_tip,
                'timestamp': time.time()
            })
    
    def recognize_dynamic_gesture(self):
        """Recognize dynamic gesture from landmark history"""
        if len(self.landmark_history) < self.window_size // 2:
            return None
        
        # Extract trajectory
        trajectory = self.extract_trajectory()
        
        # Match against templates
        best_match = None
        best_score = 0
        
        for gesture_name, template in self.gesture_templates.items():
            score = self.match_trajectory(trajectory, template)
            if score > best_score and score > 0.7:  # Threshold
                best_score = score
                best_match = gesture_name
        
        return {
            'gesture': best_match,
            'confidence': best_score,
            'trajectory': trajectory
        } if best_match else None
    
    def extract_trajectory(self):
        """Extract normalized trajectory from landmark history"""
        positions = [item['position'] for item in self.landmark_history]
        
        if len(positions) < 2:
            return []
        
        # Normalize trajectory
        positions = np.array(positions)
        
        # Center trajectory
        center = np.mean(positions, axis=0)
        centered = positions - center
        
        # Scale trajectory
        max_distance = np.max(np.linalg.norm(centered, axis=1))
        if max_distance > 0:
            normalized = centered / max_distance
        else:
            normalized = centered
        
        return normalized.tolist()
    
    def create_wave_template(self):
        """Create wave gesture template"""
        # Wave: left-right-left-right motion
        t = np.linspace(0, 4*np.pi, 30)
        x = np.sin(t) * 0.5
        y = np.zeros_like(t)
        
        return np.column_stack([x, y]).tolist()
    
    def create_swipe_left_template(self):
        """Create left swipe template"""
        # Swipe left: right to left motion
        t = np.linspace(0, 1, 20)
        x = 1 - 2*t  # From 1 to -1
        y = np.zeros_like(t)
        
        return np.column_stack([x, y]).tolist()
    
    def create_swipe_right_template(self):
        """Create right swipe template"""
        # Swipe right: left to right motion
        t = np.linspace(0, 1, 20)
        x = -1 + 2*t  # From -1 to 1
        y = np.zeros_like(t)
        
        return np.column_stack([x, y]).tolist()
    
    def create_circle_template(self):
        """Create circular gesture template"""
        # Circle: circular motion
        t = np.linspace(0, 2*np.pi, 30)
        x = np.cos(t) * 0.5
        y = np.sin(t) * 0.5
        
        return np.column_stack([x, y]).tolist()
    
    def create_come_here_template(self):
        """Create 'come here' gesture template"""
        # Come here: repeated forward motion
        template = []
        for i in range(3):  # 3 repetitions
            t = np.linspace(0, 1, 10)
            x = np.zeros_like(t)
            y = -0.5 + t  # Forward motion
            template.extend(np.column_stack([x, y]).tolist())
        
        return template
    
    def match_trajectory(self, trajectory, template):
        """Match trajectory against template using DTW"""
        if len(trajectory) == 0 or len(template) == 0:
            return 0
        
        # Simple DTW implementation
        n, m = len(trajectory), len(template)
        dtw_matrix = np.full((n+1, m+1), np.inf)
        dtw_matrix[0, 0] = 0
        
        for i in range(1, n+1):
            for j in range(1, m+1):
                cost = np.linalg.norm(np.array(trajectory[i-1]) - np.array(template[j-1]))
                dtw_matrix[i, j] = cost + min(
                    dtw_matrix[i-1, j],      # insertion
                    dtw_matrix[i, j-1],      # deletion
                    dtw_matrix[i-1, j-1]     # match
                )
        
        # Normalize score
        max_distance = max(n, m) * 2  # Maximum possible distance
        normalized_score = 1 - (dtw_matrix[n, m] / max_distance)
        
        return max(0, normalized_score)
```

## Facial Recognition and Emotion Detection

### Face Detection and Recognition

```python
import cv2
import face_recognition
import numpy as np
from deepface import DeepFace
import pickle
import os

class FacialRecognitionSystem:
    def __init__(self):
        self.known_faces = {}
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # Load known faces if database exists
        self.load_face_database()
    
    def register_person(self, name, image_path):
        """Register a new person's face"""
        
        # Load and encode face
        image = face_recognition.load_image_file(image_path)
        face_encodings = face_recognition.face_encodings(image)
        
        if len(face_encodings) > 0:
            self.known_faces[name] = face_encodings[0]
            print(f"Registered {name}")
            
            # Save to database
            self.save_face_database()
            return True
        else:
            print(f"No face found in {image_path}")
            return False
    
    def recognize_faces(self, image):
        """Recognize faces in image"""
        
        # Find face locations and encodings
        face_locations = face_recognition.face_locations(image)
        face_encodings = face_recognition.face_encodings(image, face_locations)
        
        recognized_faces = []
        
        for face_encoding, face_location in zip(face_encodings, face_locations):
            # Compare with known faces
            matches = face_recognition.compare_faces(
                list(self.known_faces.values()), face_encoding, tolerance=0.6)
            
            name = "Unknown"
            confidence = 0
            
            if True in matches:
                # Find best match
                face_distances = face_recognition.face_distance(
                    list(self.known_faces.values()), face_encoding)
                
                best_match_index = np.argmin(face_distances)
                
                if matches[best_match_index]:
                    name = list(self.known_faces.keys())[best_match_index]
                    confidence = 1 - face_distances[best_match_index]
            
            recognized_faces.append({
                'name': name,
                'confidence': confidence,
                'location': face_location,
                'encoding': face_encoding
            })
        
        return recognized_faces
    
    def detect_emotions(self, image):
        """Detect emotions in faces"""
        
        try:
            # Use DeepFace for emotion detection
            results = DeepFace.analyze(image, actions=['emotion'], enforce_detection=False)
            
            emotions = []
            
            # Handle both single face and multiple faces
            if isinstance(results, list):
                for result in results:
                    emotions.append({
                        'emotions': result['emotion'],
                        'dominant_emotion': result['dominant_emotion'],
                        'region': result['region']
                    })
            else:
                emotions.append({
                    'emotions': results['emotion'],
                    'dominant_emotion': results['dominant_emotion'],
                    'region': results['region']
                })
            
            return emotions
            
        except Exception as e:
            print(f"Emotion detection error: {e}")
            return []
    
    def analyze_face_comprehensive(self, image):
        """Comprehensive face analysis including recognition and emotions"""
        
        # Face recognition
        recognized_faces = self.recognize_faces(image)
        
        # Emotion detection
        emotions = self.detect_emotions(image)
        
        # Combine results
        combined_results = []
        
        for i, face in enumerate(recognized_faces):
            result = {
                'name': face['name'],
                'confidence': face['confidence'],
                'location': face['location'],
                'emotions': emotions[i] if i < len(emotions) else None
            }
            combined_results.append(result)
        
        return combined_results
    
    def save_face_database(self):
        """Save known faces to file"""
        with open('face_database.pkl', 'wb') as f:
            pickle.dump(self.known_faces, f)
    
    def load_face_database(self):
        """Load known faces from file"""
        if os.path.exists('face_database.pkl'):
            with open('face_database.pkl', 'rb') as f:
                self.known_faces = pickle.load(f)
            print(f"Loaded {len(self.known_faces)} known faces")
    
    def draw_face_analysis(self, image, analysis_results):
        """Draw face analysis results on image"""
        
        annotated_image = image.copy()
        
        for result in analysis_results:
            top, right, bottom, left = result['location']
            name = result['name']
            confidence = result['confidence']
            emotions = result.get('emotions')
            
            # Draw face rectangle
            color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
            cv2.rectangle(annotated_image, (left, top), (right, bottom), color, 2)
            
            # Draw name and confidence
            label = f"{name} ({confidence:.2f})"
            cv2.putText(annotated_image, label, (left, top - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Draw dominant emotion
            if emotions:
                emotion_label = emotions['dominant_emotion']
                cv2.putText(annotated_image, emotion_label, (left, bottom + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        return annotated_image

# Robot behavior adaptation based on facial analysis
class EmotionAwareRobot:
    def __init__(self):
        self.facial_system = FacialRecognitionSystem()
        self.behavior_rules = self.define_behavior_rules()
        self.interaction_history = {}
    
    def define_behavior_rules(self):
        """Define robot behavior based on detected emotions"""
        return {
            'happy': {
                'response': "You seem happy! How can I help you today?",
                'behavior': 'enthusiastic',
                'voice_tone': 'cheerful'
            },
            'sad': {
                'response': "I notice you seem sad. Is there anything I can do to help?",
                'behavior': 'gentle',
                'voice_tone': 'soft'
            },
            'angry': {
                'response': "I sense you might be frustrated. Let me know how I can assist.",
                'behavior': 'calm',
                'voice_tone': 'soothing'
            },
            'surprised': {
                'response': "You look surprised! What's happening?",
                'behavior': 'curious',
                'voice_tone': 'interested'
            },
            'fear': {
                'response': "Don't worry, I'm here to help. You're safe.",
                'behavior': 'reassuring',
                'voice_tone': 'calm'
            },
            'neutral': {
                'response': "Hello! How can I assist you?",
                'behavior': 'standard',
                'voice_tone': 'normal'
            }
        }
    
    def analyze_and_respond(self, image):
        """Analyze face and generate appropriate response"""
        
        # Analyze faces in image
        analysis_results = self.facial_system.analyze_face_comprehensive(image)
        
        responses = []
        
        for result in analysis_results:
            name = result['name']
            emotions = result.get('emotions')
            
            if emotions:
                dominant_emotion = emotions['dominant_emotion']
                
                # Get appropriate response
                behavior_rule = self.behavior_rules.get(dominant_emotion, 
                                                       self.behavior_rules['neutral'])
                
                # Personalize response if person is known
                if name != "Unknown":
                    personalized_response = f"Hello {name}! {behavior_rule['response']}"
                else:
                    personalized_response = behavior_rule['response']
                
                # Update interaction history
                self.update_interaction_history(name, dominant_emotion)
                
                responses.append({
                    'person': name,
                    'emotion': dominant_emotion,
                    'response': personalized_response,
                    'behavior': behavior_rule['behavior'],
                    'voice_tone': behavior_rule['voice_tone']
                })
        
        return responses
    
    def update_interaction_history(self, person, emotion):
        """Update interaction history for learning"""
        
        if person not in self.interaction_history:
            self.interaction_history[person] = {
                'emotions': [],
                'interactions': 0
            }
        
        self.interaction_history[person]['emotions'].append(emotion)
        self.interaction_history[person]['interactions'] += 1
        
        # Keep only recent emotions (last 10)
        if len(self.interaction_history[person]['emotions']) > 10:
            self.interaction_history[person]['emotions'] = \
                self.interaction_history[person]['emotions'][-10:]
    
    def get_person_emotional_profile(self, person):
        """Get emotional profile for a person"""
        
        if person not in self.interaction_history:
            return None
        
        emotions = self.interaction_history[person]['emotions']
        
        if not emotions:
            return None
        
        # Calculate emotion frequencies
        emotion_counts = {}
        for emotion in emotions:
            emotion_counts[emotion] = emotion_counts.get(emotion, 0) + 1
        
        # Find most common emotion
        most_common_emotion = max(emotion_counts, key=emotion_counts.get)
        
        return {
            'most_common_emotion': most_common_emotion,
            'emotion_distribution': emotion_counts,
            'total_interactions': self.interaction_history[person]['interactions']
        }

# Example usage
def demo_facial_recognition():
    """Demo facial recognition and emotion detection"""
    
    robot = EmotionAwareRobot()
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Analyze faces and emotions
        responses = robot.analyze_and_respond(frame)
        
        # Get analysis results for drawing
        analysis_results = robot.facial_system.analyze_face_comprehensive(frame)
        
        # Draw results
        annotated_frame = robot.facial_system.draw_face_analysis(frame, analysis_results)
        
        # Display responses
        for response in responses:
            print(f"Person: {response['person']}")
            print(f"Emotion: {response['emotion']}")
            print(f"Response: {response['response']}")
            print(f"Behavior: {response['behavior']}")
            print("---")
        
        cv2.imshow('Facial Recognition', annotated_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
```

## Integration with Robot Control

### Gesture-Based Robot Commands

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import threading

class GestureControlledRobot(Node):
    def __init__(self):
        super().__init__('gesture_controlled_robot')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # Initialize recognizers
        self.gesture_recognizer = HandGestureRecognizer()
        self.dynamic_gesture_recognizer = DynamicGestureRecognizer()
        self.facial_system = FacialRecognitionSystem()
        
        # Robot state
        self.current_mode = 'idle'  # idle, gesture_control, following
        self.target_person = None
        
        # Start camera thread
        self.camera_thread = threading.Thread(target=self.camera_loop)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        
        self.get_logger().info('Gesture Controlled Robot initialized')
    
    def camera_loop(self):
        """Main camera processing loop"""
        cap = cv2.VideoCapture(0)
        
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                continue
            
            # Process gestures
            gestures = self.gesture_recognizer.recognize_gesture(frame)
            
            # Add to dynamic gesture recognizer
            if gestures:
                landmarks = gestures[0]['landmarks']
                self.dynamic_gesture_recognizer.add_landmarks(landmarks)
            
            # Check for dynamic gestures
            dynamic_gesture = self.dynamic_gesture_recognizer.recognize_dynamic_gesture()
            
            # Process facial recognition
            face_analysis = self.facial_system.analyze_face_comprehensive(frame)
            
            # Execute robot commands based on recognition results
            self.process_visual_input(gestures, dynamic_gesture, face_analysis)
            
            # Display annotated frame
            annotated_frame = self.draw_all_annotations(frame, gestures, 
                                                       dynamic_gesture, face_analysis)
            cv2.imshow('Robot Vision', annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
    
    def process_visual_input(self, gestures, dynamic_gesture, face_analysis):
        """Process visual input and generate robot commands"""
        
        # Process static gestures
        for gesture_info in gestures:
            gesture = gesture_info['gesture']
            confidence = gesture_info['confidence']
            
            if confidence > 0.7:  # High confidence threshold
                self.execute_gesture_command(gesture)
        
        # Process dynamic gestures
        if dynamic_gesture and dynamic_gesture['confidence'] > 0.7:
            self.execute_dynamic_gesture_command(dynamic_gesture['gesture'])
        
        # Process facial recognition for person following
        if face_analysis:
            self.process_face_analysis(face_analysis)
    
    def execute_gesture_command(self, gesture):
        """Execute robot command based on static gesture"""
        
        cmd = Twist()
        
        if gesture == 'thumbs_up':
            # Start/resume operation
            self.current_mode = 'gesture_control'
            self.publish_status("Gesture control activated")
        
        elif gesture == 'fist':
            # Stop robot
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.current_mode = 'idle'
            self.publish_status("Robot stopped")
        
        elif gesture == 'pointing' and self.current_mode == 'gesture_control':
            # Move forward
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.publish_status("Moving forward")
        
        elif gesture == 'open_palm' and self.current_mode == 'gesture_control':
            # Stop movement
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.publish_status("Stopping")
        
        elif gesture == 'peace_sign':
            # Switch to person following mode
            self.current_mode = 'following'
            self.publish_status("Person following mode activated")
    
    def execute_dynamic_gesture_command(self, gesture):
        """Execute robot command based on dynamic gesture"""
        
        cmd = Twist()
        
        if gesture == 'wave':
            # Greeting response - turn left and right
            self.perform_greeting_dance()
        
        elif gesture == 'swipe_left' and self.current_mode == 'gesture_control':
            # Turn left
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.cmd_vel_pub.publish(cmd)
            self.publish_status("Turning left")
        
        elif gesture == 'swipe_right' and self.current_mode == 'gesture_control':
            # Turn right
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
            self.cmd_vel_pub.publish(cmd)
            self.publish_status("Turning right")
        
        elif gesture == 'circle':
            # Spin in place
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0
            self.cmd_vel_pub.publish(cmd)
            self.publish_status("Spinning")
        
        elif gesture == 'come_here':
            # Move towards gesture source
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.publish_status("Coming to you")
    
    def process_face_analysis(self, face_analysis):
        """Process facial recognition results"""
        
        if self.current_mode == 'following' and face_analysis:
            # Find target person or closest face
            target_face = None
            
            if self.target_person:
                # Look for specific person
                for face in face_analysis:
                    if face['name'] == self.target_person:
                        target_face = face
                        break
            else:
                # Follow closest face
                target_face = face_analysis[0]
                self.target_person = target_face['name']
            
            if target_face:
                self.follow_person(target_face)
    
    def follow_person(self, face_info):
        """Follow detected person"""
        
        # Get face location
        top, right, bottom, left = face_info['location']
        face_center_x = (left + right) // 2
        
        # Assume image width of 640 pixels
        image_center_x = 320
        
        cmd = Twist()
        
        # Calculate angular velocity to center person in view
        error = face_center_x - image_center_x
        angular_velocity = -error * 0.005  # Proportional control
        
        # Move forward if person is centered
        if abs(error) < 50:  # Person is centered
            cmd.linear.x = 0.2
        else:
            cmd.linear.x = 0.0
        
        cmd.angular.z = angular_velocity
        
        # Limit velocities
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
        
        self.cmd_vel_pub.publish(cmd)
        self.publish_status(f"Following {face_info['name']}")
    
    def perform_greeting_dance(self):
        """Perform greeting dance in response to wave"""
        
        # Simple greeting: turn left, then right, then center
        import time
        
        def dance_sequence():
            cmd = Twist()
            
            # Turn left
            cmd.angular.z = 0.5
            self.cmd_vel_pub.publish(cmd)
            time.sleep(1)
            
            # Turn right
            cmd.angular.z = -0.5
            self.cmd_vel_pub.publish(cmd)
            time.sleep(1)
            
            # Stop
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
        
        # Run dance in separate thread
        dance_thread = threading.Thread(target=dance_sequence)
        dance_thread.start()
        
        self.publish_status("Greeting dance!")
    
    def publish_status(self, message):
        """Publish robot status message"""
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)
        self.get_logger().info(message)
    
    def draw_all_annotations(self, frame, gestures, dynamic_gesture, face_analysis):
        """Draw all visual recognition results on frame"""
        
        annotated_frame = frame.copy()
        
        # Draw hand gestures
        if gestures:
            annotated_frame = self.gesture_recognizer.draw_landmarks(annotated_frame, gestures)
        
        # Draw dynamic gesture
        if dynamic_gesture:
            cv2.putText(annotated_frame, f"Dynamic: {dynamic_gesture['gesture']}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Draw face analysis
        if face_analysis:
            annotated_frame = self.facial_system.draw_face_analysis(annotated_frame, face_analysis)
        
        # Draw robot mode
        cv2.putText(annotated_frame, f"Mode: {self.current_mode}", 
                   (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return annotated_frame

def main():
    rclpy.init()
    robot = GestureControlledRobot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Takeaways

- Gesture recognition enables intuitive robot control through hand movements
- Static gestures provide discrete commands while dynamic gestures offer continuous control
- Facial recognition allows robots to identify and remember specific individuals
- Emotion detection enables robots to adapt their behavior to human emotional states
- MediaPipe provides robust hand and face landmark detection capabilities
- Integration with robot control systems enables natural human-robot interaction
- Proper calibration and confidence thresholds improve recognition accuracy
- Multi-modal interaction combining gestures and facial recognition enhances user experience
- Real-time processing requires optimization for embedded robot systems
- Privacy and consent considerations are important for facial recognition systems

---

**Next:** [Lecture 3: Safety and Ethics in Human-Robot Interaction](./lecture-3.md)