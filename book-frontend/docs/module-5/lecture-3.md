---
sidebar_position: 3
---

# Lecture 3: Safety and Ethics in Human-Robot Interaction

## Introduction to Robot Safety and Ethics

As robots become more integrated into human environments, ensuring **safety** and addressing **ethical concerns** becomes paramount. This lecture covers the principles, standards, and practices for safe and ethical human-robot interaction.

## Safety in Human-Robot Interaction

### Safety Standards and Regulations

#### ISO 10218: Industrial Robot Safety
```python
class IndustrialRobotSafety:
    def __init__(self):
        self.safety_zones = {
            'collaborative_workspace': {
                'max_speed': 0.25,  # m/s
                'max_force': 150,   # N
                'max_pressure': 50  # N/cm²
            },
            'restricted_space': {
                'max_speed': 1.5,   # m/s
                'requires_safety_fence': True,
                'emergency_stop_required': True
            }
        }
        
        self.safety_functions = [
            'emergency_stop',
            'protective_stop',
            'speed_monitoring',
            'workspace_monitoring',
            'force_limiting'
        ]
    
    def check_collaborative_safety(self, robot_state, human_presence):
        """Check if robot operation is safe for collaboration"""
        
        safety_violations = []
        
        # Check speed limits
        if robot_state['speed'] > self.safety_zones['collaborative_workspace']['max_speed']:
            safety_violations.append({
                'type': 'speed_violation',
                'current': robot_state['speed'],
                'limit': self.safety_zones['collaborative_workspace']['max_speed']
            })
        
        # Check force limits
        if robot_state['force'] > self.safety_zones['collaborative_workspace']['max_force']:
            safety_violations.append({
                'type': 'force_violation',
                'current': robot_state['force'],
                'limit': self.safety_zones['collaborative_workspace']['max_force']
            })
        
        # Check human proximity
        if human_presence['distance'] < robot_state['safety_distance']:
            safety_violations.append({
                'type': 'proximity_violation',
                'distance': human_presence['distance'],
                'required_distance': robot_state['safety_distance']
            })
        
        return {
            'is_safe': len(safety_violations) == 0,
            'violations': safety_violations,
            'recommended_action': self.get_safety_action(safety_violations)
        }
    
    def get_safety_action(self, violations):
        """Determine appropriate safety action"""
        
        if not violations:
            return 'continue_operation'
        
        # Prioritize by severity
        violation_types = [v['type'] for v in violations]
        
        if 'proximity_violation' in violation_types:
            return 'emergency_stop'
        elif 'force_violation' in violation_types:
            return 'protective_stop'
        elif 'speed_violation' in violation_types:
            return 'reduce_speed'
        else:
            return 'protective_stop'
```

#### ISO 15066: Collaborative Robot Safety
```python
class CollaborativeRobotSafety:
    def __init__(self):
        # Power and Force Limiting (PFL) thresholds
        self.pfl_thresholds = {
            'skull_face': {'pressure': 130, 'force': 65},
            'forehead': {'pressure': 130, 'force': 65},
            'temple': {'pressure': 110, 'force': 55},
            'neck_throat': {'pressure': 35, 'force': 35},
            'back_shoulder': {'pressure': 210, 'force': 105},
            'chest': {'pressure': 110, 'force': 55},
            'abdomen_pelvis': {'pressure': 110, 'force': 55},
            'upper_arm': {'pressure': 150, 'force': 75},
            'forearm': {'pressure': 160, 'force': 80},
            'hand_fingers': {'pressure': 140, 'force': 70}
        }
        
        self.safety_modes = [
            'safety_monitored_stop',
            'hand_guiding',
            'speed_separation_monitoring',
            'power_force_limiting'
        ]
    
    def calculate_safe_limits(self, body_part, contact_area):
        """Calculate safe force/pressure limits for body contact"""
        
        if body_part not in self.pfl_thresholds:
            # Use most conservative limits
            body_part = 'neck_throat'
        
        thresholds = self.pfl_thresholds[body_part]
        
        # Calculate maximum allowable force based on contact area
        max_pressure = thresholds['pressure']  # N/cm²
        max_force_area = max_pressure * contact_area  # N
        max_force_absolute = thresholds['force']  # N
        
        # Use the more restrictive limit
        safe_force_limit = min(max_force_area, max_force_absolute)
        
        return {
            'max_force': safe_force_limit,
            'max_pressure': max_pressure,
            'body_part': body_part,
            'contact_area': contact_area
        }
    
    def monitor_human_robot_contact(self, force_sensor_data, contact_detection):
        """Monitor and respond to human-robot contact"""
        
        if not contact_detection['contact_detected']:
            return {'status': 'no_contact', 'action': 'continue'}
        
        # Determine contact location and area
        body_part = contact_detection['estimated_body_part']
        contact_area = contact_detection['contact_area']  # cm²
        
        # Get safe limits
        safe_limits = self.calculate_safe_limits(body_part, contact_area)
        
        # Check current force/pressure
        current_force = force_sensor_data['magnitude']
        current_pressure = current_force / contact_area if contact_area > 0 else float('inf')
        
        # Determine safety status
        if current_force > safe_limits['max_force']:
            return {
                'status': 'force_exceeded',
                'action': 'emergency_stop',
                'current_force': current_force,
                'limit': safe_limits['max_force']
            }
        elif current_pressure > safe_limits['max_pressure']:
            return {
                'status': 'pressure_exceeded',
                'action': 'emergency_stop',
                'current_pressure': current_pressure,
                'limit': safe_limits['max_pressure']
            }
        else:
            return {
                'status': 'safe_contact',
                'action': 'continue_with_monitoring',
                'safety_margin': safe_limits['max_force'] - current_force
            }
```

### Human Detection and Tracking for Safety

```python
import cv2
import numpy as np
from ultralytics import YOLO
import math

class HumanSafetyMonitor:
    def __init__(self):
        # Load human detection model
        self.human_detector = YOLO('yolov8n.pt')
        
        # Safety zones (in meters)
        self.safety_zones = {
            'danger_zone': 0.5,      # Immediate stop required
            'warning_zone': 1.0,     # Reduce speed
            'monitoring_zone': 2.0   # Monitor closely
        }
        
        # Robot specifications
        self.robot_reach = 1.5  # meters
        self.robot_position = [0, 0, 0]  # x, y, z
        
    def detect_humans(self, image, depth_image=None):
        """Detect humans in camera image"""
        
        results = self.human_detector(image)
        humans = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Check if detection is a person (class 0 in COCO)
                    if int(box.cls) == 0:  # Person class
                        confidence = float(box.conf)
                        
                        if confidence > 0.5:  # Confidence threshold
                            # Get bounding box
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            
                            # Calculate distance if depth image available
                            distance = None
                            if depth_image is not None:
                                distance = self.estimate_distance(
                                    depth_image, int(x1), int(y1), int(x2), int(y2))
                            
                            humans.append({
                                'bbox': [x1, y1, x2, y2],
                                'confidence': confidence,
                                'distance': distance,
                                'center': [(x1 + x2) / 2, (y1 + y2) / 2]
                            })
        
        return humans
    
    def estimate_distance(self, depth_image, x1, y1, x2, y2):
        """Estimate distance to human using depth image"""
        
        # Extract depth values in bounding box
        roi_depth = depth_image[int(y1):int(y2), int(x1):int(x2)]
        
        # Filter out invalid depth values
        valid_depths = roi_depth[roi_depth > 0]
        
        if len(valid_depths) > 0:
            # Use median depth for robustness
            distance = np.median(valid_depths) / 1000.0  # Convert mm to meters
            return distance
        
        return None
    
    def assess_safety_risk(self, humans):
        """Assess safety risk based on human positions"""
        
        risk_assessment = {
            'overall_risk': 'safe',
            'humans_in_danger_zone': 0,
            'humans_in_warning_zone': 0,
            'closest_human_distance': float('inf'),
            'recommended_action': 'continue'
        }
        
        for human in humans:
            distance = human['distance']
            
            if distance is not None:
                # Update closest human distance
                if distance < risk_assessment['closest_human_distance']:
                    risk_assessment['closest_human_distance'] = distance
                
                # Check safety zones
                if distance <= self.safety_zones['danger_zone']:
                    risk_assessment['humans_in_danger_zone'] += 1
                    risk_assessment['overall_risk'] = 'critical'
                    risk_assessment['recommended_action'] = 'emergency_stop'
                
                elif distance <= self.safety_zones['warning_zone']:
                    risk_assessment['humans_in_warning_zone'] += 1
                    if risk_assessment['overall_risk'] != 'critical':
                        risk_assessment['overall_risk'] = 'warning'
                        risk_assessment['recommended_action'] = 'reduce_speed'
        
        return risk_assessment
    
    def calculate_safe_trajectory(self, humans, target_position):
        """Calculate safe trajectory avoiding humans"""
        
        # Simple implementation - in practice, use advanced path planning
        safe_trajectory = []
        
        # Current robot position
        current_pos = np.array(self.robot_position[:2])  # x, y only
        target_pos = np.array(target_position[:2])
        
        # Direct path
        direct_path = target_pos - current_pos
        path_length = np.linalg.norm(direct_path)
        
        if path_length == 0:
            return [current_pos.tolist()]
        
        # Check if direct path is safe
        path_safe = True
        for human in humans:
            if human['distance'] is not None:
                # Estimate human position (simplified)
                human_pos = current_pos + direct_path * 0.5  # Assume human at mid-path
                
                # Check if path passes too close to human
                if human['distance'] < self.safety_zones['warning_zone']:
                    path_safe = False
                    break
        
        if path_safe:
            # Direct path is safe
            num_waypoints = max(2, int(path_length / 0.5))  # Waypoint every 0.5m
            for i in range(num_waypoints + 1):
                t = i / num_waypoints
                waypoint = current_pos + t * direct_path
                safe_trajectory.append(waypoint.tolist())
        else:
            # Need to plan around humans - simplified avoidance
            # In practice, use RRT*, A*, or other path planning algorithms
            
            # Add waypoint to avoid humans
            avoidance_offset = np.array([1.0, 0.0])  # Move 1m to the side
            waypoint1 = current_pos + avoidance_offset
            waypoint2 = target_pos + avoidance_offset
            
            safe_trajectory = [
                current_pos.tolist(),
                waypoint1.tolist(),
                waypoint2.tolist(),
                target_pos.tolist()
            ]
        
        return safe_trajectory
    
    def generate_safety_report(self, humans, risk_assessment):
        """Generate comprehensive safety report"""
        
        report = {
            'timestamp': time.time(),
            'humans_detected': len(humans),
            'risk_level': risk_assessment['overall_risk'],
            'safety_violations': [],
            'recommendations': []
        }
        
        # Check for safety violations
        if risk_assessment['humans_in_danger_zone'] > 0:
            report['safety_violations'].append({
                'type': 'human_in_danger_zone',
                'count': risk_assessment['humans_in_danger_zone'],
                'severity': 'critical'
            })
            report['recommendations'].append('Immediate emergency stop required')
        
        if risk_assessment['humans_in_warning_zone'] > 0:
            report['safety_violations'].append({
                'type': 'human_in_warning_zone',
                'count': risk_assessment['humans_in_warning_zone'],
                'severity': 'warning'
            })
            report['recommendations'].append('Reduce robot speed and monitor closely')
        
        # Add general recommendations
        if risk_assessment['closest_human_distance'] < 3.0:
            report['recommendations'].append('Maintain visual contact with humans')
        
        return report
```

## Ethical Considerations in Robotics

### Privacy and Data Protection

```python
import hashlib
import json
from cryptography.fernet import Fernet
from datetime import datetime, timedelta
import uuid

class RobotPrivacyManager:
    def __init__(self):
        self.encryption_key = Fernet.generate_key()
        self.cipher = Fernet(self.encryption_key)
        
        # Privacy settings
        self.privacy_settings = {
            'face_recognition_enabled': False,
            'voice_recording_enabled': False,
            'behavior_tracking_enabled': False,
            'data_retention_days': 30,
            'anonymization_enabled': True
        }
        
        # Consent tracking
        self.user_consents = {}
        
    def request_consent(self, user_id, data_types):
        """Request user consent for data collection"""
        
        consent_request = {
            'user_id': user_id,
            'data_types': data_types,
            'timestamp': datetime.now().isoformat(),
            'consent_id': str(uuid.uuid4())
        }
        
        # In practice, this would show a UI dialog
        print(f"Consent request for user {user_id}:")
        print(f"Data types: {', '.join(data_types)}")
        print("Do you consent to this data collection? (y/n)")
        
        # Simulate user response
        user_response = input().lower().strip()
        
        consent_given = user_response in ['y', 'yes']
        
        # Store consent
        self.user_consents[user_id] = {
            'consent_id': consent_request['consent_id'],
            'data_types': data_types,
            'consent_given': consent_given,
            'timestamp': consent_request['timestamp'],
            'expires': (datetime.now() + timedelta(days=365)).isoformat()
        }
        
        return consent_given
    
    def check_consent(self, user_id, data_type):
        """Check if user has given consent for specific data type"""
        
        if user_id not in self.user_consents:
            return False
        
        consent = self.user_consents[user_id]
        
        # Check if consent is still valid
        if datetime.fromisoformat(consent['expires']) < datetime.now():
            return False
        
        # Check if specific data type is consented
        return (consent['consent_given'] and 
                data_type in consent['data_types'])
    
    def anonymize_data(self, personal_data):
        """Anonymize personal data"""
        
        if not self.privacy_settings['anonymization_enabled']:
            return personal_data
        
        anonymized = {}
        
        for key, value in personal_data.items():
            if key in ['name', 'face_id', 'voice_id']:
                # Hash personal identifiers
                anonymized[key] = hashlib.sha256(str(value).encode()).hexdigest()[:16]
            elif key in ['age', 'gender']:
                # Generalize demographic data
                if key == 'age':
                    age_group = self.get_age_group(value)
                    anonymized[key] = age_group
                else:
                    anonymized[key] = value
            else:
                anonymized[key] = value
        
        return anonymized
    
    def get_age_group(self, age):
        """Convert age to age group for privacy"""
        if age < 18:
            return 'minor'
        elif age < 30:
            return '18-29'
        elif age < 50:
            return '30-49'
        elif age < 70:
            return '50-69'
        else:
            return '70+'
    
    def encrypt_sensitive_data(self, data):
        """Encrypt sensitive data"""
        
        json_data = json.dumps(data)
        encrypted_data = self.cipher.encrypt(json_data.encode())
        
        return encrypted_data
    
    def decrypt_sensitive_data(self, encrypted_data):
        """Decrypt sensitive data"""
        
        decrypted_json = self.cipher.decrypt(encrypted_data).decode()
        data = json.loads(decrypted_json)
        
        return data
    
    def cleanup_expired_data(self):
        """Remove expired data according to retention policy"""
        
        retention_days = self.privacy_settings['data_retention_days']
        cutoff_date = datetime.now() - timedelta(days=retention_days)
        
        # Remove expired consents
        expired_users = []
        for user_id, consent in self.user_consents.items():
            if datetime.fromisoformat(consent['timestamp']) < cutoff_date:
                expired_users.append(user_id)
        
        for user_id in expired_users:
            del self.user_consents[user_id]
        
        print(f"Cleaned up data for {len(expired_users)} expired users")
        
        return len(expired_users)
```

### Algorithmic Bias and Fairness

```python
import numpy as np
from sklearn.metrics import confusion_matrix, classification_report
import pandas as pd

class RobotFairnessAuditor:
    def __init__(self):
        self.protected_attributes = ['gender', 'age_group', 'ethnicity', 'disability_status']
        self.fairness_metrics = {}
        
    def audit_face_recognition_bias(self, predictions, ground_truth, demographics):
        """Audit face recognition system for bias"""
        
        audit_results = {}
        
        for attribute in self.protected_attributes:
            if attribute not in demographics:
                continue
            
            # Group results by demographic attribute
            groups = demographics[attribute].unique()
            group_results = {}
            
            for group in groups:
                group_mask = demographics[attribute] == group
                group_predictions = predictions[group_mask]
                group_truth = ground_truth[group_mask]
                
                # Calculate accuracy for this group
                accuracy = np.mean(group_predictions == group_truth)
                
                # Calculate false positive/negative rates
                tn, fp, fn, tp = confusion_matrix(group_truth, group_predictions).ravel()
                
                fpr = fp / (fp + tn) if (fp + tn) > 0 else 0  # False Positive Rate
                fnr = fn / (fn + tp) if (fn + tp) > 0 else 0  # False Negative Rate
                
                group_results[group] = {
                    'accuracy': accuracy,
                    'false_positive_rate': fpr,
                    'false_negative_rate': fnr,
                    'sample_size': np.sum(group_mask)
                }
            
            audit_results[attribute] = group_results
        
        # Calculate fairness metrics
        fairness_assessment = self.calculate_fairness_metrics(audit_results)
        
        return {
            'group_performance': audit_results,
            'fairness_metrics': fairness_assessment,
            'bias_detected': self.detect_bias(fairness_assessment)
        }
    
    def calculate_fairness_metrics(self, audit_results):
        """Calculate various fairness metrics"""
        
        fairness_metrics = {}
        
        for attribute, group_results in audit_results.items():
            groups = list(group_results.keys())
            
            if len(groups) < 2:
                continue
            
            # Demographic Parity: Equal positive prediction rates
            positive_rates = [group_results[group]['accuracy'] for group in groups]
            demographic_parity = max(positive_rates) - min(positive_rates)
            
            # Equalized Odds: Equal TPR and FPR across groups
            tpr_diff = max([1 - group_results[group]['false_negative_rate'] for group in groups]) - \
                      min([1 - group_results[group]['false_negative_rate'] for group in groups])
            
            fpr_diff = max([group_results[group]['false_positive_rate'] for group in groups]) - \
                      min([group_results[group]['false_positive_rate'] for group in groups])
            
            equalized_odds = max(tpr_diff, fpr_diff)
            
            fairness_metrics[attribute] = {
                'demographic_parity_difference': demographic_parity,
                'equalized_odds_difference': equalized_odds,
                'accuracy_difference': max(positive_rates) - min(positive_rates)
            }
        
        return fairness_metrics
    
    def detect_bias(self, fairness_metrics, threshold=0.1):
        """Detect if bias exists based on fairness metrics"""
        
        bias_detected = {}
        
        for attribute, metrics in fairness_metrics.items():
            bias_indicators = []
            
            if metrics['demographic_parity_difference'] > threshold:
                bias_indicators.append('demographic_parity_violation')
            
            if metrics['equalized_odds_difference'] > threshold:
                bias_indicators.append('equalized_odds_violation')
            
            if metrics['accuracy_difference'] > threshold:
                bias_indicators.append('accuracy_disparity')
            
            bias_detected[attribute] = {
                'bias_present': len(bias_indicators) > 0,
                'bias_types': bias_indicators,
                'severity': 'high' if len(bias_indicators) > 1 else 'medium' if len(bias_indicators) == 1 else 'low'
            }
        
        return bias_detected
    
    def generate_bias_mitigation_recommendations(self, bias_results):
        """Generate recommendations for bias mitigation"""
        
        recommendations = []
        
        for attribute, bias_info in bias_results.items():
            if bias_info['bias_present']:
                
                if 'demographic_parity_violation' in bias_info['bias_types']:
                    recommendations.append({
                        'attribute': attribute,
                        'issue': 'Unequal treatment across demographic groups',
                        'recommendation': 'Implement demographic parity constraints in model training',
                        'priority': 'high'
                    })
                
                if 'equalized_odds_violation' in bias_info['bias_types']:
                    recommendations.append({
                        'attribute': attribute,
                        'issue': 'Unequal error rates across groups',
                        'recommendation': 'Use post-processing techniques to equalize error rates',
                        'priority': 'high'
                    })
                
                if 'accuracy_disparity' in bias_info['bias_types']:
                    recommendations.append({
                        'attribute': attribute,
                        'issue': 'Significant accuracy differences between groups',
                        'recommendation': 'Collect more diverse training data and use data augmentation',
                        'priority': 'medium'
                    })
        
        return recommendations
```

### Transparency and Explainability

```python
import lime
import lime.lime_image
from lime.wrappers.scikit_image import SegmentationAlgorithm
import matplotlib.pyplot as plt

class RobotDecisionExplainer:
    def __init__(self):
        self.explanation_methods = {
            'lime': self.explain_with_lime,
            'feature_importance': self.explain_feature_importance,
            'decision_tree': self.explain_decision_tree
        }
        
    def explain_robot_decision(self, decision_type, input_data, model, method='lime'):
        """Explain robot decision using specified method"""
        
        if method not in self.explanation_methods:
            raise ValueError(f"Unknown explanation method: {method}")
        
        explanation = self.explanation_methods[method](input_data, model, decision_type)
        
        # Generate human-readable explanation
        readable_explanation = self.generate_readable_explanation(
            explanation, decision_type)
        
        return {
            'decision_type': decision_type,
            'explanation_method': method,
            'technical_explanation': explanation,
            'readable_explanation': readable_explanation
        }
    
    def explain_with_lime(self, input_data, model, decision_type):
        """Explain decision using LIME"""
        
        if decision_type == 'image_classification':
            # For image-based decisions (e.g., object recognition)
            explainer = lime.lime_image.LimeImageExplainer()
            
            def predict_fn(images):
                return model.predict(images)
            
            explanation = explainer.explain_instance(
                input_data, predict_fn, top_labels=5, num_samples=1000)
            
            return explanation
        
        elif decision_type == 'tabular_classification':
            # For structured data decisions
            explainer = lime.lime_tabular.LimeTabularExplainer(
                input_data, mode='classification')
            
            explanation = explainer.explain_instance(
                input_data[0], model.predict_proba, num_features=10)
            
            return explanation
        
        else:
            return None
    
    def explain_feature_importance(self, input_data, model, decision_type):
        """Explain decision using feature importance"""
        
        if hasattr(model, 'feature_importances_'):
            importances = model.feature_importances_
            
            # Get feature names (if available)
            feature_names = getattr(model, 'feature_names_', 
                                  [f'feature_{i}' for i in range(len(importances))])
            
            # Sort by importance
            importance_pairs = list(zip(feature_names, importances))
            importance_pairs.sort(key=lambda x: x[1], reverse=True)
            
            return {
                'method': 'feature_importance',
                'importances': importance_pairs[:10],  # Top 10 features
                'total_features': len(importances)
            }
        
        return None
    
    def generate_readable_explanation(self, explanation, decision_type):
        """Generate human-readable explanation"""
        
        if decision_type == 'object_recognition':
            return self.explain_object_recognition(explanation)
        elif decision_type == 'navigation_decision':
            return self.explain_navigation_decision(explanation)
        elif decision_type == 'safety_decision':
            return self.explain_safety_decision(explanation)
        else:
            return "I made this decision based on my analysis of the input data."
    
    def explain_object_recognition(self, explanation):
        """Explain object recognition decision"""
        
        if hasattr(explanation, 'top_labels'):
            top_label = explanation.top_labels[0]
            confidence = explanation.predict_proba[top_label]
            
            return f"I identified this as a {explanation.class_names[top_label]} " \
                   f"with {confidence:.1%} confidence. The key visual features " \
                   f"that led to this decision include the shape, color, and texture " \
                   f"patterns I detected in the image."
        
        return "I analyzed the visual features in the image to make this identification."
    
    def explain_navigation_decision(self, explanation):
        """Explain navigation decision"""
        
        return "I chose this path based on factors including obstacle avoidance, " \
               "efficiency, and safety considerations. I analyzed the environment " \
               "and selected the route that best balances these priorities."
    
    def explain_safety_decision(self, explanation):
        """Explain safety-related decision"""
        
        return "I made this safety decision to protect humans in my vicinity. " \
               "I detected potential risks and took appropriate action according " \
               "to my safety protocols."
    
    def log_decision_explanation(self, decision, explanation):
        """Log decision and explanation for audit trail"""
        
        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'decision': decision,
            'explanation': explanation,
            'decision_id': str(uuid.uuid4())
        }
        
        # In practice, this would be stored in a database
        print(f"Decision logged: {log_entry['decision_id']}")
        
        return log_entry['decision_id']
```

## Implementing Ethical Guidelines

### Robot Ethics Framework

```python
class RobotEthicsFramework:
    def __init__(self):
        self.ethical_principles = {
            'autonomy': 'Respect human autonomy and decision-making',
            'beneficence': 'Act in ways that benefit humans',
            'non_maleficence': 'Do no harm to humans',
            'justice': 'Treat all humans fairly and equally',
            'transparency': 'Be transparent about capabilities and limitations',
            'accountability': 'Maintain clear responsibility chains'
        }
        
        self.ethical_rules = self.define_ethical_rules()
        
    def define_ethical_rules(self):
        """Define specific ethical rules for robot behavior"""
        
        return {
            'human_override': {
                'description': 'Humans must always be able to override robot decisions',
                'implementation': self.check_human_override_capability,
                'priority': 'critical'
            },
            
            'informed_consent': {
                'description': 'Obtain informed consent before data collection',
                'implementation': self.check_informed_consent,
                'priority': 'high'
            },
            
            'harm_prevention': {
                'description': 'Prevent physical and psychological harm to humans',
                'implementation': self.assess_harm_potential,
                'priority': 'critical'
            },
            
            'privacy_protection': {
                'description': 'Protect human privacy and personal data',
                'implementation': self.check_privacy_compliance,
                'priority': 'high'
            },
            
            'fair_treatment': {
                'description': 'Treat all humans fairly regardless of demographics',
                'implementation': self.check_fair_treatment,
                'priority': 'high'
            }
        }
    
    def evaluate_ethical_compliance(self, action, context):
        """Evaluate if proposed action complies with ethical guidelines"""
        
        compliance_results = {}
        
        for rule_name, rule_info in self.ethical_rules.items():
            try:
                compliance = rule_info['implementation'](action, context)
                compliance_results[rule_name] = {
                    'compliant': compliance['compliant'],
                    'explanation': compliance['explanation'],
                    'priority': rule_info['priority']
                }
            except Exception as e:
                compliance_results[rule_name] = {
                    'compliant': False,
                    'explanation': f'Error evaluating rule: {str(e)}',
                    'priority': rule_info['priority']
                }
        
        # Determine overall compliance
        critical_violations = [rule for rule, result in compliance_results.items() 
                             if not result['compliant'] and result['priority'] == 'critical']
        
        overall_compliant = len(critical_violations) == 0
        
        return {
            'overall_compliant': overall_compliant,
            'rule_compliance': compliance_results,
            'critical_violations': critical_violations,
            'recommendation': self.get_ethical_recommendation(compliance_results)
        }
    
    def check_human_override_capability(self, action, context):
        """Check if human can override the action"""
        
        # Check if emergency stop is available
        emergency_stop_available = context.get('emergency_stop_available', False)
        
        # Check if action is reversible
        action_reversible = action.get('reversible', False)
        
        # Check if human is present and able to intervene
        human_present = context.get('human_present', False)
        
        compliant = emergency_stop_available and (action_reversible or human_present)
        
        explanation = f"Emergency stop: {emergency_stop_available}, " \
                     f"Reversible: {action_reversible}, " \
                     f"Human present: {human_present}"
        
        return {
            'compliant': compliant,
            'explanation': explanation
        }
    
    def check_informed_consent(self, action, context):
        """Check if informed consent was obtained"""
        
        requires_consent = action.get('requires_consent', False)
        
        if not requires_consent:
            return {
                'compliant': True,
                'explanation': 'Action does not require consent'
            }
        
        consent_obtained = context.get('consent_obtained', False)
        consent_valid = context.get('consent_valid', False)
        
        compliant = consent_obtained and consent_valid
        
        return {
            'compliant': compliant,
            'explanation': f'Consent obtained: {consent_obtained}, Valid: {consent_valid}'
        }
    
    def assess_harm_potential(self, action, context):
        """Assess potential for harm"""
        
        # Physical harm assessment
        physical_risk = action.get('physical_risk_level', 'low')
        
        # Psychological harm assessment
        psychological_risk = action.get('psychological_risk_level', 'low')
        
        # Check if appropriate safeguards are in place
        safeguards_active = context.get('safety_systems_active', False)
        
        high_risk = physical_risk == 'high' or psychological_risk == 'high'
        compliant = not high_risk or safeguards_active
        
        return {
            'compliant': compliant,
            'explanation': f'Physical risk: {physical_risk}, '
                          f'Psychological risk: {psychological_risk}, '
                          f'Safeguards: {safeguards_active}'
        }
    
    def check_privacy_compliance(self, action, context):
        """Check privacy compliance"""
        
        collects_personal_data = action.get('collects_personal_data', False)
        
        if not collects_personal_data:
            return {
                'compliant': True,
                'explanation': 'Action does not collect personal data'
            }
        
        privacy_policy_shown = context.get('privacy_policy_shown', False)
        data_minimization = action.get('data_minimization', False)
        encryption_enabled = context.get('encryption_enabled', False)
        
        compliant = privacy_policy_shown and data_minimization and encryption_enabled
        
        return {
            'compliant': compliant,
            'explanation': f'Privacy policy: {privacy_policy_shown}, '
                          f'Data minimization: {data_minimization}, '
                          f'Encryption: {encryption_enabled}'
        }
    
    def check_fair_treatment(self, action, context):
        """Check for fair treatment"""
        
        # Check if action treats all users equally
        differential_treatment = action.get('differential_treatment', False)
        
        if differential_treatment:
            # Check if differential treatment is justified
            justification = action.get('differential_treatment_justification', '')
            justified_reasons = ['accessibility', 'safety', 'medical_necessity']
            
            justified = any(reason in justification.lower() for reason in justified_reasons)
            
            return {
                'compliant': justified,
                'explanation': f'Differential treatment justified: {justified} ({justification})'
            }
        
        return {
            'compliant': True,
            'explanation': 'Equal treatment for all users'
        }
    
    def get_ethical_recommendation(self, compliance_results):
        """Get recommendation based on compliance results"""
        
        critical_violations = [rule for rule, result in compliance_results.items() 
                             if not result['compliant'] and result['priority'] == 'critical']
        
        if critical_violations:
            return f"STOP: Critical ethical violations detected: {', '.join(critical_violations)}"
        
        high_violations = [rule for rule, result in compliance_results.items() 
                          if not result['compliant'] and result['priority'] == 'high']
        
        if high_violations:
            return f"CAUTION: High-priority ethical concerns: {', '.join(high_violations)}"
        
        return "PROCEED: Action is ethically compliant"
```

## Key Takeaways

- Safety standards like ISO 10218 and ISO 15066 provide guidelines for human-robot interaction
- Collaborative robots must implement power and force limiting to ensure human safety
- Human detection and tracking systems are essential for maintaining safe operation zones
- Privacy protection requires consent management, data anonymization, and encryption
- Algorithmic bias must be actively monitored and mitigated in robot AI systems
- Transparency and explainability help build trust in robot decision-making
- Ethical frameworks provide structured approaches to evaluating robot behavior
- Human override capabilities must always be maintained in robot systems
- Regular auditing and compliance checking ensure ongoing ethical operation
- Balancing functionality with safety and ethics is an ongoing challenge in robotics

---

**Next:** [Lecture 4: Conversational AI and Multimodal Interaction](./lecture-4.md)