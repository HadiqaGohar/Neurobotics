---
sidebar_position: 4
---

# Lecture 4: Conversational AI and Multimodal Interaction

## Introduction to Conversational AI for Robots

**Conversational AI** enables robots to engage in natural, contextual dialogues with humans, combining speech, text, gestures, and visual cues for rich multimodal interactions. This creates more intuitive and engaging human-robot experiences.

## Advanced Dialogue Systems

### Context-Aware Conversation Management

```python
import openai
from datetime import datetime, timedelta
import json
import numpy as np
from collections import deque
import threading
import time

class AdvancedDialogueManager:
    def __init__(self, openai_api_key):
        openai.api_key = openai_api_key
        
        # Conversation context
        self.conversation_history = deque(maxlen=20)
        self.user_profile = {}
        self.current_context = {
            'location': None,
            'task': None,
            'emotional_state': 'neutral',
            'urgency_level': 'normal'
        }
        
        # Memory systems
        self.short_term_memory = deque(maxlen=10)
        self.long_term_memory = {}
        self.episodic_memory = []
        
        # Personality and behavior
        self.robot_personality = {
            'name': 'Assistant',
            'personality_traits': ['helpful', 'friendly', 'patient'],
            'communication_style': 'professional_casual',
            'humor_level': 'mild',
            'formality_level': 'medium'
        }
        
    def process_conversation_turn(self, user_input, multimodal_context=None):
        """Process a complete conversation turn with multimodal input"""
        
        # Update context with multimodal information
        if multimodal_context:
            self.update_context_from_multimodal(multimodal_context)
        
        # Analyze user input
        input_analysis = self.analyze_user_input(user_input)
        
        # Update user profile and memory
        self.update_user_profile(input_analysis)
        self.update_memory_systems(user_input, input_analysis)
        
        # Generate contextual response
        response = self.generate_contextual_response(user_input, input_analysis)
        
        # Add to conversation history
        self.add_to_conversation_history('user', user_input, input_analysis)
        self.add_to_conversation_history('robot', response['text'], response)
        
        return response
    
    def analyze_user_input(self, user_input):
        """Analyze user input for intent, emotion, and context"""
        
        # Use GPT for comprehensive analysis
        analysis_prompt = f"""
        Analyze the following user input for a robot assistant:
        User: "{user_input}"
        
        Provide analysis in JSON format:
        {{
            "intent": "primary intent (e.g., request_help, ask_question, give_command)",
            "emotion": "detected emotion (happy, sad, frustrated, neutral, etc.)",
            "urgency": "urgency level (low, normal, high, critical)",
            "topics": ["list", "of", "main", "topics"],
            "entities": {{"person": [], "object": [], "location": [], "time": []}},
            "requires_action": true/false,
            "conversation_type": "task_oriented/social/informational"
        }}
        """
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": analysis_prompt}],
                temperature=0.3
            )
            
            analysis = json.loads(response.choices[0].message.content)
            return analysis
            
        except Exception as e:
            print(f"Error in input analysis: {e}")
            return {
                "intent": "unknown",
                "emotion": "neutral",
                "urgency": "normal",
                "topics": [],
                "entities": {},
                "requires_action": False,
                "conversation_type": "informational"
            }
    
    def update_context_from_multimodal(self, multimodal_context):
        """Update conversation context from multimodal inputs"""
        
        # Visual context (from computer vision)
        if 'visual' in multimodal_context:
            visual = multimodal_context['visual']
            
            # Update location context
            if 'scene_type' in visual:
                self.current_context['location'] = visual['scene_type']
            
            # Update emotional state from facial recognition
            if 'detected_emotion' in visual:
                self.current_context['emotional_state'] = visual['detected_emotion']
            
            # Update objects in view
            if 'objects_detected' in visual:
                self.current_context['visible_objects'] = visual['objects_detected']
        
        # Audio context (from speech analysis)
        if 'audio' in multimodal_context:
            audio = multimodal_context['audio']
            
            # Update emotional state from voice tone
            if 'voice_emotion' in audio:
                self.current_context['voice_emotion'] = audio['voice_emotion']
            
            # Update urgency from speech patterns
            if 'speech_urgency' in audio:
                self.current_context['urgency_level'] = audio['speech_urgency']
        
        # Gesture context
        if 'gestures' in multimodal_context:
            gestures = multimodal_context['gestures']
            self.current_context['recent_gestures'] = gestures
    
    def update_user_profile(self, input_analysis):
        """Update user profile based on interaction"""
        
        # Initialize profile if new user
        if not self.user_profile:
            self.user_profile = {
                'interaction_count': 0,
                'preferred_topics': {},
                'communication_style': 'unknown',
                'typical_emotions': [],
                'common_requests': [],
                'relationship_level': 'new'
            }
        
        # Update interaction count
        self.user_profile['interaction_count'] += 1
        
        # Update preferred topics
        for topic in input_analysis.get('topics', []):
            if topic in self.user_profile['preferred_topics']:
                self.user_profile['preferred_topics'][topic] += 1
            else:
                self.user_profile['preferred_topics'][topic] = 1
        
        # Update emotional patterns
        emotion = input_analysis.get('emotion', 'neutral')
        self.user_profile['typical_emotions'].append(emotion)
        
        # Keep only recent emotions (last 20)
        if len(self.user_profile['typical_emotions']) > 20:
            self.user_profile['typical_emotions'] = self.user_profile['typical_emotions'][-20:]
        
        # Update relationship level
        if self.user_profile['interaction_count'] > 50:
            self.user_profile['relationship_level'] = 'close'
        elif self.user_profile['interaction_count'] > 10:
            self.user_profile['relationship_level'] = 'familiar'
        elif self.user_profile['interaction_count'] > 3:
            self.user_profile['relationship_level'] = 'acquainted'
    
    def update_memory_systems(self, user_input, input_analysis):
        """Update different memory systems"""
        
        # Short-term memory (recent interactions)
        self.short_term_memory.append({
            'input': user_input,
            'analysis': input_analysis,
            'timestamp': datetime.now().isoformat(),
            'context': self.current_context.copy()
        })
        
        # Long-term memory (important facts and preferences)
        if input_analysis.get('intent') == 'provide_information':
            # Extract and store important information
            entities = input_analysis.get('entities', {})
            for entity_type, entity_list in entities.items():
                if entity_type not in self.long_term_memory:
                    self.long_term_memory[entity_type] = {}
                
                for entity in entity_list:
                    self.long_term_memory[entity_type][entity] = {
                        'mentioned_count': self.long_term_memory[entity_type].get(entity, {}).get('mentioned_count', 0) + 1,
                        'last_mentioned': datetime.now().isoformat(),
                        'context': input_analysis.get('topics', [])
                    }
        
        # Episodic memory (significant events)
        if input_analysis.get('urgency') in ['high', 'critical'] or \
           input_analysis.get('emotion') in ['very_happy', 'very_sad', 'angry']:
            
            episode = {
                'type': 'significant_interaction',
                'input': user_input,
                'analysis': input_analysis,
                'timestamp': datetime.now().isoformat(),
                'context': self.current_context.copy(),
                'significance_score': self.calculate_significance_score(input_analysis)
            }
            
            self.episodic_memory.append(episode)
            
            # Keep only most significant episodes (max 100)
            if len(self.episodic_memory) > 100:
                self.episodic_memory.sort(key=lambda x: x['significance_score'], reverse=True)
                self.episodic_memory = self.episodic_memory[:100]
    
    def calculate_significance_score(self, input_analysis):
        """Calculate significance score for episodic memory"""
        
        score = 0
        
        # Urgency contributes to significance
        urgency_scores = {'low': 1, 'normal': 2, 'high': 4, 'critical': 8}
        score += urgency_scores.get(input_analysis.get('urgency', 'normal'), 2)
        
        # Strong emotions are significant
        emotional_scores = {
            'very_happy': 6, 'very_sad': 6, 'angry': 7, 'frustrated': 5,
            'excited': 4, 'worried': 4, 'happy': 2, 'sad': 2, 'neutral': 1
        }
        score += emotional_scores.get(input_analysis.get('emotion', 'neutral'), 1)
        
        # Action requirements add significance
        if input_analysis.get('requires_action', False):
            score += 3
        
        return score
    
    def generate_contextual_response(self, user_input, input_analysis):
        """Generate contextually appropriate response"""
        
        # Build context for GPT
        context_prompt = self.build_context_prompt(user_input, input_analysis)
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.get_system_prompt()},
                    {"role": "user", "content": context_prompt}
                ],
                temperature=0.7,
                max_tokens=200
            )
            
            response_text = response.choices[0].message.content
            
            # Add multimodal response elements
            multimodal_response = self.add_multimodal_elements(response_text, input_analysis)
            
            return {
                'text': response_text,
                'multimodal': multimodal_response,
                'context_used': self.current_context.copy(),
                'response_type': input_analysis.get('conversation_type', 'informational')
            }
            
        except Exception as e:
            print(f"Error generating response: {e}")
            return {
                'text': "I'm sorry, I'm having trouble processing that right now. Could you please try again?",
                'multimodal': {},
                'context_used': {},
                'response_type': 'error'
            }
    
    def build_context_prompt(self, user_input, input_analysis):
        """Build comprehensive context prompt for GPT"""
        
        # Recent conversation history
        recent_history = list(self.conversation_history)[-6:]  # Last 3 exchanges
        history_text = "\n".join([f"{item['speaker']}: {item['content']}" 
                                 for item in recent_history])
        
        # User profile summary
        profile_summary = self.get_user_profile_summary()
        
        # Current context
        context_summary = f"""
        Current Context:
        - Location: {self.current_context.get('location', 'unknown')}
        - User emotion: {self.current_context.get('emotional_state', 'neutral')}
        - Urgency: {input_analysis.get('urgency', 'normal')}
        - Visible objects: {', '.join(self.current_context.get('visible_objects', []))}
        """
        
        # Relevant memories
        relevant_memories = self.retrieve_relevant_memories(input_analysis)
        memory_text = "\n".join([f"- {memory}" for memory in relevant_memories])
        
        prompt = f"""
        Recent conversation:
        {history_text}
        
        {profile_summary}
        
        {context_summary}
        
        Relevant memories:
        {memory_text}
        
        Current user input: "{user_input}"
        
        Input analysis: {json.dumps(input_analysis, indent=2)}
        
        Please provide an appropriate response that:
        1. Addresses the user's intent and emotion
        2. Uses the conversation context and history
        3. Matches the robot's personality
        4. Is helpful and engaging
        """
        
        return prompt
    
    def get_system_prompt(self):
        """Get system prompt defining robot personality and behavior"""
        
        personality = self.robot_personality
        
        return f"""
        You are {personality['name']}, a helpful robot assistant with the following characteristics:
        
        Personality traits: {', '.join(personality['personality_traits'])}
        Communication style: {personality['communication_style']}
        Humor level: {personality['humor_level']}
        Formality level: {personality['formality_level']}
        
        Guidelines:
        - Be helpful, friendly, and patient
        - Adapt your tone to the user's emotional state
        - Use context from previous conversations
        - Be concise but thorough
        - Show empathy when appropriate
        - Maintain consistency with your personality
        - If you need to perform actions, clearly state what you will do
        
        Always respond in a natural, conversational way that feels authentic to your personality.
        """
    
    def get_user_profile_summary(self):
        """Get summary of user profile for context"""
        
        if not self.user_profile:
            return "User profile: New user, no previous interactions."
        
        # Most common topics
        top_topics = sorted(self.user_profile['preferred_topics'].items(), 
                           key=lambda x: x[1], reverse=True)[:3]
        
        # Most common emotions
        emotion_counts = {}
        for emotion in self.user_profile['typical_emotions']:
            emotion_counts[emotion] = emotion_counts.get(emotion, 0) + 1
        
        common_emotions = sorted(emotion_counts.items(), 
                               key=lambda x: x[1], reverse=True)[:2]
        
        return f"""
        User profile:
        - Interactions: {self.user_profile['interaction_count']}
        - Relationship level: {self.user_profile['relationship_level']}
        - Preferred topics: {', '.join([topic for topic, count in top_topics])}
        - Typical emotions: {', '.join([emotion for emotion, count in common_emotions])}
        """
    
    def retrieve_relevant_memories(self, input_analysis):
        """Retrieve relevant memories for current context"""
        
        relevant_memories = []
        
        # Check short-term memory for related topics
        for memory in self.short_term_memory:
            memory_topics = memory['analysis'].get('topics', [])
            current_topics = input_analysis.get('topics', [])
            
            # If topics overlap, include memory
            if any(topic in memory_topics for topic in current_topics):
                relevant_memories.append(f"Recently discussed: {memory['input'][:50]}...")
        
        # Check long-term memory for mentioned entities
        entities = input_analysis.get('entities', {})
        for entity_type, entity_list in entities.items():
            if entity_type in self.long_term_memory:
                for entity in entity_list:
                    if entity in self.long_term_memory[entity_type]:
                        memory_info = self.long_term_memory[entity_type][entity]
                        relevant_memories.append(
                            f"I remember {entity} (mentioned {memory_info['mentioned_count']} times)")
        
        # Check episodic memory for similar contexts
        for episode in self.episodic_memory[-10:]:  # Recent significant episodes
            episode_topics = episode['analysis'].get('topics', [])
            current_topics = input_analysis.get('topics', [])
            
            if any(topic in episode_topics for topic in current_topics):
                relevant_memories.append(f"Previous significant interaction: {episode['input'][:50]}...")
        
        return relevant_memories[:5]  # Limit to 5 most relevant
    
    def add_multimodal_elements(self, response_text, input_analysis):
        """Add multimodal elements to response"""
        
        multimodal_elements = {}
        
        # Determine appropriate gestures
        if input_analysis.get('emotion') == 'happy':
            multimodal_elements['gesture'] = 'thumbs_up'
        elif input_analysis.get('emotion') in ['sad', 'frustrated']:
            multimodal_elements['gesture'] = 'sympathetic_nod'
        elif input_analysis.get('intent') == 'greeting':
            multimodal_elements['gesture'] = 'wave'
        
        # Determine voice tone
        emotion = input_analysis.get('emotion', 'neutral')
        if emotion in ['happy', 'excited']:
            multimodal_elements['voice_tone'] = 'cheerful'
        elif emotion in ['sad', 'worried']:
            multimodal_elements['voice_tone'] = 'gentle'
        elif emotion in ['angry', 'frustrated']:
            multimodal_elements['voice_tone'] = 'calm'
        else:
            multimodal_elements['voice_tone'] = 'neutral'
        
        # Determine movement behavior
        urgency = input_analysis.get('urgency', 'normal')
        if urgency == 'high':
            multimodal_elements['movement'] = 'quick_response'
        elif urgency == 'critical':
            multimodal_elements['movement'] = 'immediate_action'
        else:
            multimodal_elements['movement'] = 'normal'
        
        # Add visual displays if appropriate
        if 'show' in response_text.lower() or 'display' in response_text.lower():
            multimodal_elements['visual_display'] = 'information_screen'
        
        return multimodal_elements
    
    def add_to_conversation_history(self, speaker, content, analysis=None):
        """Add entry to conversation history"""
        
        entry = {
            'speaker': speaker,
            'content': content,
            'timestamp': datetime.now().isoformat(),
            'analysis': analysis
        }
        
        self.conversation_history.append(entry)
```

### Multimodal Fusion and Response Generation

```python
import cv2
import numpy as np
from transformers import BlipProcessor, BlipForConditionalGeneration
import torch
import speech_recognition as sr
import pyttsx3
import threading
import queue

class MultimodalInteractionSystem:
    def __init__(self):
        # Initialize components
        self.dialogue_manager = AdvancedDialogueManager(openai_api_key="your-key")
        
        # Vision components
        self.vision_processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-base")
        self.vision_model = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-base")
        
        # Speech components
        self.speech_recognizer = sr.Recognizer()
        self.tts_engine = pyttsx3.init()
        
        # Gesture recognition (from previous lecture)
        self.gesture_recognizer = HandGestureRecognizer()
        
        # Multimodal fusion
        self.fusion_weights = {
            'speech': 0.4,
            'vision': 0.3,
            'gesture': 0.2,
            'context': 0.1
        }
        
        # Input queues for different modalities
        self.speech_queue = queue.Queue()
        self.vision_queue = queue.Queue()
        self.gesture_queue = queue.Queue()
        
        # Start processing threads
        self.start_multimodal_processing()
    
    def start_multimodal_processing(self):
        """Start processing threads for different modalities"""
        
        # Speech processing thread
        speech_thread = threading.Thread(target=self.process_speech_stream)
        speech_thread.daemon = True
        speech_thread.start()
        
        # Vision processing thread
        vision_thread = threading.Thread(target=self.process_vision_stream)
        vision_thread.daemon = True
        vision_thread.start()
        
        # Gesture processing thread
        gesture_thread = threading.Thread(target=self.process_gesture_stream)
        gesture_thread.daemon = True
        gesture_thread.start()
        
        # Fusion and response thread
        fusion_thread = threading.Thread(target=self.multimodal_fusion_loop)
        fusion_thread.daemon = True
        fusion_thread.start()
    
    def process_speech_stream(self):
        """Process continuous speech input"""
        
        microphone = sr.Microphone()
        
        with microphone as source:
            self.speech_recognizer.adjust_for_ambient_noise(source)
        
        while True:
            try:
                with microphone as source:
                    # Listen for speech with timeout
                    audio = self.speech_recognizer.listen(source, timeout=1, phrase_time_limit=5)
                
                # Recognize speech
                text = self.speech_recognizer.recognize_google(audio)
                
                # Add to speech queue with timestamp
                speech_input = {
                    'type': 'speech',
                    'content': text,
                    'timestamp': time.time(),
                    'confidence': 0.8  # Google Speech API doesn't provide confidence
                }
                
                self.speech_queue.put(speech_input)
                
            except sr.WaitTimeoutError:
                pass  # No speech detected
            except sr.UnknownValueError:
                pass  # Speech not understood
            except Exception as e:
                print(f"Speech recognition error: {e}")
                time.sleep(1)
    
    def process_vision_stream(self):
        """Process continuous vision input"""
        
        cap = cv2.VideoCapture(0)
        
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            
            try:
                # Generate image caption
                inputs = self.vision_processor(frame, return_tensors="pt")
                out = self.vision_model.generate(**inputs, max_length=50)
                caption = self.vision_processor.decode(out[0], skip_special_tokens=True)
                
                # Detect objects and scenes
                objects_detected = self.detect_objects_in_scene(frame)
                scene_type = self.classify_scene(frame)
                
                vision_input = {
                    'type': 'vision',
                    'content': {
                        'caption': caption,
                        'objects': objects_detected,
                        'scene_type': scene_type,
                        'frame': frame
                    },
                    'timestamp': time.time(),
                    'confidence': 0.7
                }
                
                self.vision_queue.put(vision_input)
                
            except Exception as e:
                print(f"Vision processing error: {e}")
            
            time.sleep(0.5)  # Process at 2 FPS
    
    def process_gesture_stream(self):
        """Process continuous gesture input"""
        
        cap = cv2.VideoCapture(0)
        
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            
            try:
                # Recognize gestures
                gestures = self.gesture_recognizer.recognize_gesture(frame)
                
                if gestures:
                    gesture_input = {
                        'type': 'gesture',
                        'content': gestures,
                        'timestamp': time.time(),
                        'confidence': max([g['confidence'] for g in gestures])
                    }
                    
                    self.gesture_queue.put(gesture_input)
                
            except Exception as e:
                print(f"Gesture processing error: {e}")
            
            time.sleep(0.1)  # Process at 10 FPS
    
    def multimodal_fusion_loop(self):
        """Main fusion loop that combines multimodal inputs"""
        
        multimodal_buffer = {
            'speech': [],
            'vision': [],
            'gesture': []
        }
        
        while True:
            current_time = time.time()
            
            # Collect recent inputs from all modalities
            self.collect_recent_inputs(multimodal_buffer, current_time)
            
            # Check if we have enough input to process
            if self.should_process_multimodal_input(multimodal_buffer):
                
                # Fuse multimodal inputs
                fused_input = self.fuse_multimodal_inputs(multimodal_buffer)
                
                # Generate response
                response = self.generate_multimodal_response(fused_input)
                
                # Execute response
                self.execute_multimodal_response(response)
                
                # Clear processed inputs
                self.clear_processed_inputs(multimodal_buffer, current_time)
            
            time.sleep(0.1)  # Check every 100ms
    
    def collect_recent_inputs(self, buffer, current_time):
        """Collect recent inputs from all modality queues"""
        
        # Collect speech inputs
        while not self.speech_queue.empty():
            try:
                speech_input = self.speech_queue.get_nowait()
                buffer['speech'].append(speech_input)
            except queue.Empty:
                break
        
        # Collect vision inputs
        while not self.vision_queue.empty():
            try:
                vision_input = self.vision_queue.get_nowait()
                buffer['vision'].append(vision_input)
            except queue.Empty:
                break
        
        # Collect gesture inputs
        while not self.gesture_queue.empty():
            try:
                gesture_input = self.gesture_queue.get_nowait()
                buffer['gesture'].append(gesture_input)
            except queue.Empty:
                break
        
        # Remove old inputs (older than 5 seconds)
        for modality in buffer:
            buffer[modality] = [
                inp for inp in buffer[modality] 
                if current_time - inp['timestamp'] < 5.0
            ]
    
    def should_process_multimodal_input(self, buffer):
        """Determine if we should process current multimodal input"""
        
        # Process if we have speech input
        if buffer['speech']:
            return True
        
        # Process if we have high-confidence gesture
        for gesture_input in buffer['gesture']:
            if gesture_input['confidence'] > 0.8:
                return True
        
        # Process if we have significant visual change
        if len(buffer['vision']) > 1:
            # Check for scene changes
            recent_scenes = [v['content']['scene_type'] for v in buffer['vision'][-2:]]
            if len(set(recent_scenes)) > 1:  # Scene changed
                return True
        
        return False
    
    def fuse_multimodal_inputs(self, buffer):
        """Fuse inputs from different modalities"""
        
        fused_input = {
            'primary_modality': None,
            'primary_content': None,
            'supporting_context': {},
            'confidence': 0,
            'timestamp': time.time()
        }
        
        # Determine primary modality (highest weighted confidence)
        modality_scores = {}
        
        if buffer['speech']:
            latest_speech = buffer['speech'][-1]
            modality_scores['speech'] = (
                latest_speech['confidence'] * self.fusion_weights['speech']
            )
        
        if buffer['gesture']:
            latest_gesture = max(buffer['gesture'], key=lambda x: x['confidence'])
            modality_scores['gesture'] = (
                latest_gesture['confidence'] * self.fusion_weights['gesture']
            )
        
        if buffer['vision']:
            latest_vision = buffer['vision'][-1]
            modality_scores['vision'] = (
                latest_vision['confidence'] * self.fusion_weights['vision']
            )
        
        # Select primary modality
        if modality_scores:
            primary_modality = max(modality_scores, key=modality_scores.get)
            fused_input['primary_modality'] = primary_modality
            fused_input['confidence'] = modality_scores[primary_modality]
            
            # Set primary content
            if primary_modality == 'speech':
                fused_input['primary_content'] = buffer['speech'][-1]['content']
            elif primary_modality == 'gesture':
                best_gesture = max(buffer['gesture'], key=lambda x: x['confidence'])
                fused_input['primary_content'] = best_gesture['content']
            elif primary_modality == 'vision':
                fused_input['primary_content'] = buffer['vision'][-1]['content']
        
        # Add supporting context from other modalities
        if buffer['vision']:
            fused_input['supporting_context']['visual'] = {
                'scene': buffer['vision'][-1]['content']['scene_type'],
                'objects': buffer['vision'][-1]['content']['objects'],
                'caption': buffer['vision'][-1]['content']['caption']
            }
        
        if buffer['gesture']:
            recent_gestures = [g['content'] for g in buffer['gesture'][-3:]]
            fused_input['supporting_context']['gestures'] = recent_gestures
        
        return fused_input
    
    def generate_multimodal_response(self, fused_input):
        """Generate response based on fused multimodal input"""
        
        if fused_input['primary_modality'] == 'speech':
            # Process as conversational input
            multimodal_context = {
                'visual': fused_input['supporting_context'].get('visual', {}),
                'gestures': fused_input['supporting_context'].get('gestures', [])
            }
            
            response = self.dialogue_manager.process_conversation_turn(
                fused_input['primary_content'], multimodal_context)
            
        elif fused_input['primary_modality'] == 'gesture':
            # Process gesture command
            response = self.process_gesture_command(
                fused_input['primary_content'], 
                fused_input['supporting_context'])
            
        elif fused_input['primary_modality'] == 'vision':
            # Process visual scene
            response = self.process_visual_scene(
                fused_input['primary_content'],
                fused_input['supporting_context'])
            
        else:
            response = {
                'text': "I'm not sure what you want me to do.",
                'multimodal': {'voice_tone': 'confused'}
            }
        
        return response
    
    def process_gesture_command(self, gesture_content, context):
        """Process gesture-based command"""
        
        gesture_responses = {
            'thumbs_up': "Thank you! I'm glad you're satisfied.",
            'pointing': "I see you're pointing. What would you like me to look at?",
            'wave': "Hello! Nice to see you!",
            'peace_sign': "Peace! How can I help you today?",
            'fist': "I understand you want me to stop.",
            'open_palm': "I see your open hand. Are you asking me to wait?"
        }
        
        if gesture_content and len(gesture_content) > 0:
            gesture_name = gesture_content[0]['gesture']
            response_text = gesture_responses.get(
                gesture_name, 
                f"I see your {gesture_name} gesture. How can I help?"
            )
        else:
            response_text = "I noticed your gesture. What can I do for you?"
        
        return {
            'text': response_text,
            'multimodal': {
                'gesture': 'acknowledgment_nod',
                'voice_tone': 'friendly'
            }
        }
    
    def process_visual_scene(self, visual_content, context):
        """Process visual scene information"""
        
        scene_type = visual_content.get('scene_type', 'unknown')
        objects = visual_content.get('objects', [])
        caption = visual_content.get('caption', '')
        
        if objects:
            object_list = ', '.join(objects[:3])  # Mention top 3 objects
            response_text = f"I can see {object_list} in this {scene_type}. {caption}"
        else:
            response_text = f"I'm looking at {caption}"
        
        return {
            'text': response_text,
            'multimodal': {
                'gesture': 'looking_around',
                'voice_tone': 'observational'
            }
        }
    
    def execute_multimodal_response(self, response):
        """Execute multimodal response"""
        
        # Speak the text response
        self.speak_response(response['text'], response.get('multimodal', {}))
        
        # Execute gesture if specified
        multimodal = response.get('multimodal', {})
        if 'gesture' in multimodal:
            self.execute_gesture(multimodal['gesture'])
        
        # Execute movement if specified
        if 'movement' in multimodal:
            self.execute_movement(multimodal['movement'])
        
        # Display visual information if specified
        if 'visual_display' in multimodal:
            self.show_visual_display(multimodal['visual_display'])
    
    def speak_response(self, text, multimodal_info):
        """Speak response with appropriate voice characteristics"""
        
        voice_tone = multimodal_info.get('voice_tone', 'neutral')
        
        # Adjust TTS parameters based on tone
        if voice_tone == 'cheerful':
            self.tts_engine.setProperty('rate', 200)  # Faster
            self.tts_engine.setProperty('volume', 0.9)  # Louder
        elif voice_tone == 'gentle':
            self.tts_engine.setProperty('rate', 150)  # Slower
            self.tts_engine.setProperty('volume', 0.7)  # Softer
        elif voice_tone == 'calm':
            self.tts_engine.setProperty('rate', 160)  # Moderate
            self.tts_engine.setProperty('volume', 0.8)  # Moderate
        else:  # neutral
            self.tts_engine.setProperty('rate', 180)  # Normal
            self.tts_engine.setProperty('volume', 0.8)  # Normal
        
        # Speak the text
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()
    
    def execute_gesture(self, gesture_name):
        """Execute robot gesture"""
        
        # This would interface with robot hardware
        print(f"Executing gesture: {gesture_name}")
        
        # Placeholder for actual gesture execution
        gesture_commands = {
            'acknowledgment_nod': 'nod_head',
            'thumbs_up': 'raise_thumb',
            'wave': 'wave_hand',
            'looking_around': 'turn_head_scan',
            'sympathetic_nod': 'slow_nod'
        }
        
        command = gesture_commands.get(gesture_name, 'default_gesture')
        # send_robot_command(command)
    
    def execute_movement(self, movement_type):
        """Execute robot movement"""
        
        print(f"Executing movement: {movement_type}")
        
        # Placeholder for actual movement execution
        movement_commands = {
            'quick_response': 'increase_response_speed',
            'immediate_action': 'emergency_response_mode',
            'normal': 'standard_operation_mode'
        }
        
        command = movement_commands.get(movement_type, 'normal')
        # send_robot_command(command)
    
    def show_visual_display(self, display_type):
        """Show visual information on robot display"""
        
        print(f"Showing visual display: {display_type}")
        
        # Placeholder for actual display control
        if display_type == 'information_screen':
            # Show relevant information on robot's screen
            pass
    
    def detect_objects_in_scene(self, frame):
        """Detect objects in the current frame"""
        
        # Placeholder for object detection
        # In practice, use YOLO or similar
        return ['person', 'chair', 'table']
    
    def classify_scene(self, frame):
        """Classify the type of scene"""
        
        # Placeholder for scene classification
        # In practice, use scene classification model
        return 'indoor_office'
    
    def clear_processed_inputs(self, buffer, current_time):
        """Clear inputs that have been processed"""
        
        # Keep only very recent inputs for context
        for modality in buffer:
            buffer[modality] = [
                inp for inp in buffer[modality] 
                if current_time - inp['timestamp'] < 2.0
            ]

# Example usage
def demo_multimodal_interaction():
    """Demo multimodal interaction system"""
    
    system = MultimodalInteractionSystem()
    
    print("Multimodal interaction system started!")
    print("The system is now listening for speech, watching for gestures, and analyzing the visual scene.")
    print("Try speaking, waving, or showing objects to the camera.")
    
    try:
        # Keep the system running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down multimodal interaction system...")

# Uncomment to run demo
# demo_multimodal_interaction()
```

## Key Takeaways

- Advanced dialogue systems maintain context, memory, and user profiles for natural conversations
- Multimodal fusion combines speech, vision, and gesture inputs for richer interaction
- Context-aware responses adapt to user emotions, urgency, and environmental factors
- Memory systems (short-term, long-term, episodic) enable continuity across interactions
- Personality modeling makes robot interactions more engaging and consistent
- Real-time processing requires efficient threading and queue management
- Confidence scoring helps determine the most reliable input modality
- Multimodal responses include speech, gestures, movements, and visual displays
- User profiling enables personalized and adaptive robot behavior
- Proper error handling ensures robust operation in real-world conditions

---

**Next:** [Lecture 5: Future of Human-Robot Collaboration](./lecture-5.md)