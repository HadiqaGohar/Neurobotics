---
sidebar_position: 1
---

# Lecture 1: Natural Language Processing for Robots

## Introduction to NLP in Robotics

**Natural Language Processing (NLP)** enables robots to understand, interpret, and respond to human language, making human-robot interaction more intuitive and natural. This capability transforms robots from command-driven machines into conversational partners.

## Why NLP Matters for Robots

### Traditional vs. Natural Language Interaction

| Traditional Interface | Natural Language Interface |
|----------------------|----------------------------|
| **Commands**: "MOVE_FORWARD 2.5" | **Speech**: "Please move forward a bit" |
| **Programming**: Complex code | **Conversation**: "Can you help me?" |
| **Learning curve**: Technical training required | **Intuitive**: Natural human communication |
| **Flexibility**: Limited predefined commands | **Adaptive**: Understands variations and context |

### Applications in Robotics

#### 1. Voice Commands and Control
- **Simple commands**: "Stop", "Go home", "Follow me"
- **Complex instructions**: "Pick up the red box and place it on the table"
- **Contextual requests**: "Bring me what I was working on yesterday"

#### 2. Human-Robot Collaboration
- **Task coordination**: "Let's work together on this assembly"
- **Status updates**: "I'm having trouble with this part"
- **Learning from feedback**: "That's not quite right, try again"

#### 3. Service and Assistance
- **Information queries**: "What's the weather like today?"
- **Navigation help**: "Where is the nearest coffee shop?"
- **Task assistance**: "Help me organize these documents"

## Core NLP Components for Robotics

### 1. Speech Recognition (Speech-to-Text)

Converting spoken language into text that robots can process.

#### OpenAI Whisper Integration

```python
import whisper
import pyaudio
import wave
import numpy as np
import threading
import queue

class RobotSpeechRecognizer:
    def __init__(self, model_size="base"):
        """Initialize Whisper model for speech recognition"""
        self.model = whisper.load_model(model_size)
        self.audio_queue = queue.Queue()
        self.is_listening = False
        
        # Audio recording parameters
        self.chunk_size = 1024
        self.sample_rate = 16000
        self.channels = 1
        self.record_seconds = 5
        
    def start_listening(self):
        """Start continuous speech recognition"""
        self.is_listening = True
        
        # Start audio recording thread
        recording_thread = threading.Thread(target=self._record_audio)
        recording_thread.daemon = True
        recording_thread.start()
        
        # Start processing thread
        processing_thread = threading.Thread(target=self._process_audio)
        processing_thread.daemon = True
        processing_thread.start()
        
        print("Speech recognition started. Say something...")
    
    def stop_listening(self):
        """Stop speech recognition"""
        self.is_listening = False
        print("Speech recognition stopped.")
    
    def _record_audio(self):
        """Record audio in chunks"""
        audio = pyaudio.PyAudio()
        
        stream = audio.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )
        
        while self.is_listening:
            try:
                # Record audio chunk
                frames = []
                for _ in range(0, int(self.sample_rate / self.chunk_size * self.record_seconds)):
                    if not self.is_listening:
                        break
                    data = stream.read(self.chunk_size)
                    frames.append(data)
                
                if frames:
                    # Convert to numpy array
                    audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
                    audio_data = audio_data.astype(np.float32) / 32768.0
                    
                    # Add to processing queue
                    self.audio_queue.put(audio_data)
                    
            except Exception as e:
                print(f"Audio recording error: {e}")
        
        stream.stop_stream()
        stream.close()
        audio.terminate()
    
    def _process_audio(self):
        """Process audio chunks with Whisper"""
        while self.is_listening:
            try:
                # Get audio from queue (with timeout)
                audio_data = self.audio_queue.get(timeout=1.0)
                
                # Transcribe with Whisper
                result = self.model.transcribe(audio_data)
                text = result["text"].strip()
                
                if text:
                    print(f"Recognized: {text}")
                    self.on_speech_recognized(text)
                    
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Speech processing error: {e}")
    
    def on_speech_recognized(self, text):
        """Override this method to handle recognized speech"""
        pass
    
    def transcribe_file(self, audio_file_path):
        """Transcribe audio file"""
        result = self.model.transcribe(audio_file_path)
        return result["text"]

# Example usage
class RobotVoiceInterface(RobotSpeechRecognizer):
    def __init__(self):
        super().__init__(model_size="base")
        self.command_processor = RobotCommandProcessor()
    
    def on_speech_recognized(self, text):
        """Process recognized speech"""
        print(f"Processing command: {text}")
        
        # Process the command
        response = self.command_processor.process_command(text)
        
        # Speak the response (text-to-speech)
        self.speak_response(response)
    
    def speak_response(self, text):
        """Convert text to speech (placeholder)"""
        print(f"Robot says: {text}")
        # Implement text-to-speech here
```

### 2. Natural Language Understanding (NLU)

Extracting meaning and intent from recognized text.

#### Intent Recognition and Entity Extraction

```python
import re
import spacy
from transformers import pipeline
import json

class RobotNLU:
    def __init__(self):
        # Load spaCy model for NER and parsing
        self.nlp = spacy.load("en_core_web_sm")
        
        # Load pre-trained intent classifier
        self.intent_classifier = pipeline(
            "text-classification",
            model="microsoft/DialoGPT-medium"
        )
        
        # Define robot-specific intents and patterns
        self.intent_patterns = {
            'move': [
                r'(go|move|walk|drive|navigate) (to|towards|forward|backward|left|right)',
                r'(come|go) (here|there|over there)',
                r'(follow|chase) (me|him|her|it)'
            ],
            'pick_up': [
                r'(pick up|grab|take|get) (the|a|an) (\w+)',
                r'(lift|raise) (the|a|an) (\w+)',
                r'(bring me|fetch) (the|a|an) (\w+)'
            ],
            'place': [
                r'(put|place|set|drop) (it|the \w+) (on|in|at) (the|a|an) (\w+)',
                r'(move|transfer) (it|the \w+) (to|onto) (the|a|an) (\w+)'
            ],
            'stop': [
                r'(stop|halt|pause|wait)',
                r'(don\'t|do not) (move|go|continue)'
            ],
            'question': [
                r'(what|where|when|how|why|who) (is|are|was|were|do|does|did|can|could|will|would)',
                r'(can you|could you|will you|would you) (\w+)',
                r'(tell me|show me) (about|how to|where)'
            ],
            'greeting': [
                r'(hello|hi|hey|good morning|good afternoon|good evening)',
                r'(how are you|how\'s it going|what\'s up)'
            ]
        }
        
        # Define entity types
        self.entity_types = {
            'object': ['box', 'cup', 'book', 'phone', 'key', 'bottle', 'pen'],
            'location': ['table', 'shelf', 'floor', 'kitchen', 'bedroom', 'office'],
            'person': ['me', 'you', 'him', 'her', 'john', 'mary', 'user'],
            'direction': ['left', 'right', 'forward', 'backward', 'up', 'down'],
            'color': ['red', 'blue', 'green', 'yellow', 'black', 'white', 'orange']
        }
    
    def analyze_text(self, text):
        """Comprehensive NLU analysis"""
        text = text.lower().strip()
        
        # Extract intent
        intent = self.extract_intent(text)
        
        # Extract entities
        entities = self.extract_entities(text)
        
        # Parse with spaCy for additional linguistic features
        doc = self.nlp(text)
        
        # Extract grammatical information
        grammar_info = self.extract_grammar_info(doc)
        
        return {
            'text': text,
            'intent': intent,
            'entities': entities,
            'grammar': grammar_info,
            'confidence': self.calculate_confidence(intent, entities)
        }
    
    def extract_intent(self, text):
        """Extract intent from text using pattern matching"""
        
        for intent, patterns in self.intent_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text, re.IGNORECASE):
                    return {
                        'name': intent,
                        'confidence': 0.9,  # High confidence for pattern match
                        'pattern': pattern
                    }
        
        # Fallback: use transformer model for intent classification
        try:
            result = self.intent_classifier(text)
            return {
                'name': 'unknown',
                'confidence': 0.5,
                'transformer_result': result
            }
        except:
            return {
                'name': 'unknown',
                'confidence': 0.1
            }
    
    def extract_entities(self, text):
        """Extract entities using spaCy NER and custom patterns"""
        doc = self.nlp(text)
        entities = []
        
        # spaCy named entities
        for ent in doc.ents:
            entities.append({
                'text': ent.text,
                'label': ent.label_,
                'start': ent.start_char,
                'end': ent.end_char,
                'source': 'spacy'
            })
        
        # Custom entity extraction
        words = text.split()
        for word in words:
            for entity_type, entity_list in self.entity_types.items():
                if word in entity_list:
                    entities.append({
                        'text': word,
                        'label': entity_type,
                        'source': 'custom'
                    })
        
        return entities
    
    def extract_grammar_info(self, doc):
        """Extract grammatical information"""
        return {
            'tokens': [{'text': token.text, 'pos': token.pos_, 'dep': token.dep_} 
                      for token in doc],
            'noun_phrases': [chunk.text for chunk in doc.noun_chunks],
            'verb_phrases': [token.text for token in doc if token.pos_ == 'VERB']
        }
    
    def calculate_confidence(self, intent, entities):
        """Calculate overall confidence score"""
        intent_conf = intent.get('confidence', 0)
        entity_conf = 0.8 if entities else 0.3
        
        return (intent_conf + entity_conf) / 2

class RobotCommandProcessor:
    def __init__(self):
        self.nlu = RobotNLU()
        self.robot_state = {
            'position': [0, 0, 0],
            'holding_object': None,
            'last_command': None
        }
    
    def process_command(self, text):
        """Process natural language command"""
        
        # Analyze the text
        analysis = self.nlu.analyze_text(text)
        
        print(f"NLU Analysis: {json.dumps(analysis, indent=2)}")
        
        # Generate robot action based on intent
        action = self.intent_to_action(analysis)
        
        # Execute action (placeholder)
        response = self.execute_action(action)
        
        return response
    
    def intent_to_action(self, analysis):
        """Convert intent and entities to robot action"""
        intent = analysis['intent']['name']
        entities = analysis['entities']
        
        # Extract relevant entities
        objects = [e['text'] for e in entities if e['label'] == 'object']
        locations = [e['text'] for e in entities if e['label'] == 'location']
        directions = [e['text'] for e in entities if e['label'] == 'direction']
        colors = [e['text'] for e in entities if e['label'] == 'color']
        
        if intent == 'move':
            if directions:
                return {
                    'type': 'move',
                    'direction': directions[0],
                    'target': locations[0] if locations else None
                }
            elif locations:
                return {
                    'type': 'navigate',
                    'target': locations[0]
                }
        
        elif intent == 'pick_up':
            target_object = None
            if colors and objects:
                target_object = f"{colors[0]} {objects[0]}"
            elif objects:
                target_object = objects[0]
            
            return {
                'type': 'pick_up',
                'object': target_object
            }
        
        elif intent == 'place':
            return {
                'type': 'place',
                'object': self.robot_state['holding_object'],
                'location': locations[0] if locations else 'table'
            }
        
        elif intent == 'stop':
            return {
                'type': 'stop'
            }
        
        elif intent == 'question':
            return {
                'type': 'answer_question',
                'question': analysis['text']
            }
        
        elif intent == 'greeting':
            return {
                'type': 'greeting'
            }
        
        else:
            return {
                'type': 'unknown',
                'original_text': analysis['text']
            }
    
    def execute_action(self, action):
        """Execute robot action and return response"""
        
        action_type = action['type']
        
        if action_type == 'move':
            direction = action.get('direction', 'forward')
            return f"Moving {direction}"
        
        elif action_type == 'navigate':
            target = action['target']
            return f"Navigating to {target}"
        
        elif action_type == 'pick_up':
            obj = action['object']
            if obj:
                self.robot_state['holding_object'] = obj
                return f"Picking up {obj}"
            else:
                return "I don't see what you want me to pick up"
        
        elif action_type == 'place':
            obj = action['object']
            location = action['location']
            if obj:
                self.robot_state['holding_object'] = None
                return f"Placing {obj} on {location}"
            else:
                return "I'm not holding anything to place"
        
        elif action_type == 'stop':
            return "Stopping all movement"
        
        elif action_type == 'answer_question':
            return self.answer_question(action['question'])
        
        elif action_type == 'greeting':
            return "Hello! How can I help you today?"
        
        else:
            return "I'm sorry, I didn't understand that command"
    
    def answer_question(self, question):
        """Answer simple questions about robot state"""
        
        if 'holding' in question or 'carrying' in question:
            if self.robot_state['holding_object']:
                return f"I'm holding {self.robot_state['holding_object']}"
            else:
                return "I'm not holding anything"
        
        elif 'where' in question and ('you' in question or 'robot' in question):
            pos = self.robot_state['position']
            return f"I'm at position ({pos[0]}, {pos[1]}, {pos[2]})"
        
        elif 'what' in question and 'do' in question:
            return "I can move around, pick up objects, and answer questions"
        
        else:
            return "I'm not sure how to answer that question"

# Example usage
def demo_robot_nlu():
    """Demonstrate robot NLU capabilities"""
    
    processor = RobotCommandProcessor()
    
    test_commands = [
        "Please move forward",
        "Pick up the red box",
        "Go to the kitchen",
        "Put it on the table",
        "Stop moving",
        "What are you holding?",
        "Hello robot",
        "Can you help me with this task?"
    ]
    
    for command in test_commands:
        print(f"\nUser: {command}")
        response = processor.process_command(command)
        print(f"Robot: {response}")

# Run demo
demo_robot_nlu()
```

### 3. Dialogue Management

Managing conversation flow and context in multi-turn interactions.

```python
import json
from datetime import datetime
from enum import Enum

class DialogueState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    WAITING_CONFIRMATION = "waiting_confirmation"
    EXECUTING_TASK = "executing_task"
    ASKING_CLARIFICATION = "asking_clarification"

class RobotDialogueManager:
    def __init__(self):
        self.state = DialogueState.IDLE
        self.conversation_history = []
        self.current_task = None
        self.context = {
            'user_preferences': {},
            'mentioned_objects': [],
            'mentioned_locations': [],
            'pending_actions': []
        }
        self.clarification_needed = None
        
    def process_user_input(self, user_input, nlu_analysis):
        """Process user input based on current dialogue state"""
        
        # Add to conversation history
        self.add_to_history('user', user_input, nlu_analysis)
        
        # Process based on current state
        if self.state == DialogueState.IDLE:
            response = self.handle_idle_state(nlu_analysis)
        
        elif self.state == DialogueState.WAITING_CONFIRMATION:
            response = self.handle_confirmation_state(nlu_analysis)
        
        elif self.state == DialogueState.ASKING_CLARIFICATION:
            response = self.handle_clarification_state(nlu_analysis)
        
        else:
            response = self.handle_general_input(nlu_analysis)
        
        # Add response to history
        self.add_to_history('robot', response['text'], response)
        
        return response
    
    def handle_idle_state(self, nlu_analysis):
        """Handle input when robot is idle"""
        intent = nlu_analysis['intent']['name']
        
        if intent in ['move', 'pick_up', 'place']:
            # Check if we have enough information
            missing_info = self.check_missing_information(nlu_analysis)
            
            if missing_info:
                self.state = DialogueState.ASKING_CLARIFICATION
                self.clarification_needed = missing_info
                return {
                    'text': self.generate_clarification_question(missing_info),
                    'action': 'ask_clarification',
                    'state': self.state
                }
            else:
                # We have enough info, ask for confirmation
                self.current_task = self.create_task_from_analysis(nlu_analysis)
                self.state = DialogueState.WAITING_CONFIRMATION
                return {
                    'text': f"Should I {self.describe_task(self.current_task)}?",
                    'action': 'request_confirmation',
                    'state': self.state
                }
        
        elif intent == 'question':
            return {
                'text': self.answer_question(nlu_analysis),
                'action': 'answer',
                'state': self.state
            }
        
        elif intent == 'greeting':
            return {
                'text': "Hello! I'm ready to help. What would you like me to do?",
                'action': 'greet',
                'state': self.state
            }
        
        else:
            return {
                'text': "I'm ready to help. You can ask me to move, pick up objects, or answer questions.",
                'action': 'prompt',
                'state': self.state
            }
    
    def handle_confirmation_state(self, nlu_analysis):
        """Handle confirmation responses"""
        text = nlu_analysis['text'].lower()
        
        if any(word in text for word in ['yes', 'yeah', 'ok', 'okay', 'sure', 'go ahead']):
            # User confirmed, execute task
            self.state = DialogueState.EXECUTING_TASK
            return {
                'text': f"Executing: {self.describe_task(self.current_task)}",
                'action': 'execute_task',
                'task': self.current_task,
                'state': self.state
            }
        
        elif any(word in text for word in ['no', 'nope', 'cancel', 'stop', 'never mind']):
            # User cancelled
            self.state = DialogueState.IDLE
            self.current_task = None
            return {
                'text': "Task cancelled. What else can I help you with?",
                'action': 'cancel',
                'state': self.state
            }
        
        else:
            # Unclear response, ask again
            return {
                'text': f"I'm not sure. Should I {self.describe_task(self.current_task)}? Please say yes or no.",
                'action': 'request_confirmation',
                'state': self.state
            }
    
    def handle_clarification_state(self, nlu_analysis):
        """Handle clarification responses"""
        
        # Extract the missing information from user's response
        entities = nlu_analysis['entities']
        
        # Update context with new information
        for entity in entities:
            if entity['label'] == 'object':
                self.context['mentioned_objects'].append(entity['text'])
            elif entity['label'] == 'location':
                self.context['mentioned_locations'].append(entity['text'])
        
        # Check if we now have enough information
        missing_info = self.check_missing_information_with_context(self.clarification_needed)
        
        if missing_info:
            # Still missing information
            return {
                'text': self.generate_clarification_question(missing_info),
                'action': 'ask_clarification',
                'state': self.state
            }
        else:
            # Now we have enough info
            self.current_task = self.create_task_from_context()
            self.state = DialogueState.WAITING_CONFIRMATION
            self.clarification_needed = None
            
            return {
                'text': f"Got it! Should I {self.describe_task(self.current_task)}?",
                'action': 'request_confirmation',
                'state': self.state
            }
    
    def check_missing_information(self, nlu_analysis):
        """Check what information is missing for the task"""
        intent = nlu_analysis['intent']['name']
        entities = nlu_analysis['entities']
        
        objects = [e['text'] for e in entities if e['label'] == 'object']
        locations = [e['text'] for e in entities if e['label'] == 'location']
        
        missing = []
        
        if intent == 'pick_up' and not objects:
            missing.append('object')
        
        elif intent == 'place' and not locations:
            missing.append('location')
        
        elif intent == 'move' and not locations:
            missing.append('destination')
        
        return missing
    
    def generate_clarification_question(self, missing_info):
        """Generate appropriate clarification question"""
        
        if 'object' in missing_info:
            return "What object would you like me to pick up?"
        
        elif 'location' in missing_info:
            return "Where would you like me to place it?"
        
        elif 'destination' in missing_info:
            return "Where would you like me to go?"
        
        else:
            return "Could you please provide more details?"
    
    def create_task_from_analysis(self, nlu_analysis):
        """Create task object from NLU analysis"""
        intent = nlu_analysis['intent']['name']
        entities = nlu_analysis['entities']
        
        task = {
            'type': intent,
            'timestamp': datetime.now().isoformat(),
            'parameters': {}
        }
        
        for entity in entities:
            if entity['label'] not in task['parameters']:
                task['parameters'][entity['label']] = []
            task['parameters'][entity['label']].append(entity['text'])
        
        return task
    
    def describe_task(self, task):
        """Generate human-readable task description"""
        task_type = task['type']
        params = task['parameters']
        
        if task_type == 'pick_up':
            obj = params.get('object', ['something'])[0]
            return f"pick up the {obj}"
        
        elif task_type == 'place':
            location = params.get('location', ['table'])[0]
            return f"place it on the {location}"
        
        elif task_type == 'move':
            destination = params.get('location', ['there'])[0]
            return f"move to the {destination}"
        
        else:
            return f"perform {task_type}"
    
    def add_to_history(self, speaker, text, analysis=None):
        """Add interaction to conversation history"""
        entry = {
            'speaker': speaker,
            'text': text,
            'timestamp': datetime.now().isoformat(),
            'analysis': analysis
        }
        
        self.conversation_history.append(entry)
        
        # Keep only last 20 interactions
        if len(self.conversation_history) > 20:
            self.conversation_history = self.conversation_history[-20:]
    
    def get_conversation_summary(self):
        """Get summary of recent conversation"""
        recent_interactions = self.conversation_history[-5:]
        
        summary = {
            'current_state': self.state.value,
            'current_task': self.current_task,
            'recent_interactions': recent_interactions,
            'context': self.context
        }
        
        return summary

# Example usage with complete dialogue system
class RobotDialogueSystem:
    def __init__(self):
        self.nlu = RobotNLU()
        self.dialogue_manager = RobotDialogueManager()
        self.speech_recognizer = RobotSpeechRecognizer()
        
    def process_speech_input(self, audio_input):
        """Process speech input through complete pipeline"""
        
        # Speech to text
        text = self.speech_recognizer.transcribe_audio(audio_input)
        
        # Natural language understanding
        nlu_analysis = self.nlu.analyze_text(text)
        
        # Dialogue management
        response = self.dialogue_manager.process_user_input(text, nlu_analysis)
        
        return response
    
    def chat_interface(self):
        """Simple text-based chat interface for testing"""
        print("Robot Dialogue System Ready!")
        print("Type 'quit' to exit")
        
        while True:
            user_input = input("\nYou: ")
            
            if user_input.lower() == 'quit':
                break
            
            # Process input
            nlu_analysis = self.nlu.analyze_text(user_input)
            response = self.dialogue_manager.process_user_input(user_input, nlu_analysis)
            
            print(f"Robot: {response['text']}")
            print(f"State: {response['state'].value}")

# Demo the dialogue system
def demo_dialogue_system():
    system = RobotDialogueSystem()
    system.chat_interface()

# Uncomment to run demo
# demo_dialogue_system()
```

## Key Takeaways

- Natural Language Processing enables intuitive human-robot communication
- Speech recognition converts spoken language to text for robot processing
- Natural Language Understanding extracts meaning, intent, and entities from text
- Dialogue management maintains conversation context and flow
- Intent recognition identifies what the user wants the robot to do
- Entity extraction identifies objects, locations, and other important information
- Context management enables multi-turn conversations and clarification
- Proper error handling and clarification improve user experience
- Integration with robot control systems enables natural language commands
- Continuous learning and adaptation improve NLP performance over time

---

**Next:** [Lecture 2: Gesture and Facial Recognition](./lecture-2.md)