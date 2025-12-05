# Floating Chatbot UI with Interactive Features

A complete floating chatbot implementation integrated into the Docusaurus book-frontend with a FastAPI backend.

## Features

### 🎯 Core Features
- **Floating Icon**: Fixed bottom-right corner with smooth animations
- **Resizable Chat Window**: Toggle between small and large sizes
- **Chat History**: Persistent conversation history per session
- **Scrollable Messages**: Auto-scroll to latest messages
- **Real-time Responses**: Powered by Gemini AI model

### 🎤 Interactive Controls
- **Voice Input**: Click microphone to record voice messages
- **Text Copy**: Copy any message to clipboard with one click
- **Ask AI**: Enhanced AI help mode for detailed explanations
- **Send Messages**: Standard text input with Enter key support

### 📱 Design & Accessibility
- **Responsive Design**: Works on desktop and mobile devices
- **Smooth Animations**: Slide-up transitions and hover effects
- **Accessibility**: ARIA labels, focus management, keyboard navigation
- **High Contrast Support**: Adapts to user preferences
- **Reduced Motion**: Respects user motion preferences

## Project Structure

```
book-frontend/src/components/ChatBot/
├── index.tsx              # Main ChatBot component
├── ChatWindow.tsx         # Chat window container
├── FloatingIcon.tsx       # Floating chat icon
├── Message.tsx            # Individual message component
├── MessageList.tsx        # Messages container with welcome screen
├── MessageInput.tsx       # Text input component
├── VoiceInput.tsx         # Voice recording component
├── ChatBotWrapper.tsx     # Docusaurus SSR wrapper
├── api.ts                 # API client for backend communication
├── types.ts               # TypeScript type definitions
└── styles.css             # Complete CSS styling

book-frontend/src/theme/
└── Root.tsx               # Global theme integration

book-backend/
└── main.py                # FastAPI server with chatbot endpoints
```

## Setup Instructions

### Backend Setup

1. **Navigate to backend directory:**
   ```bash
   cd book-backend
   ```

2. **Install dependencies:**
   ```bash
   uv sync
   ```

3. **Set up environment variables:**
   Create a `.env` file with your Gemini API key:
   ```env
   GEMINI_API_KEY=your_gemini_api_key_here
   ```

4. **Start the backend server:**
   ```bash
   uv run python main.py
   ```
   
   The API will be available at `http://localhost:8000`

### Frontend Setup

1. **Navigate to frontend directory:**
   ```bash
   cd book-frontend
   ```

2. **Install dependencies:**
   ```bash
   npm install
   ```

3. **Start the development server:**
   ```bash
   npm start
   ```
   
   The frontend will be available at `http://localhost:3000`

## API Endpoints

### Chat Endpoints

- **POST /chat** - Send a message and get AI response
- **GET /chat/history/{session_id}** - Get chat history for a session
- **POST /chat/voice** - Process voice input (mock implementation)
- **POST /chat/ask-ai** - Enhanced AI help responses

### Request/Response Examples

**Send Message:**
```json
POST /chat
{
  "message": "Hello, how are you?",
  "session_id": "optional_session_id"
}

Response:
{
  "message": {
    "id": "msg_123",
    "content": "Hello! I'm doing well, thank you for asking. How can I help you today?",
    "sender": "ai",
    "timestamp": "2024-12-04T10:30:00Z",
    "session_id": "session_123"
  },
  "session_id": "session_123"
}
```

## Usage Guide

### Basic Chat
1. Click the floating chat icon in the bottom-right corner
2. Type your message in the input field
3. Press Enter or click the send button
4. View AI responses in real-time

### Voice Input
1. Click the microphone icon in the input area
2. Allow microphone permissions when prompted
3. Speak your message (recording indicator will show)
4. Click the microphone again to stop recording
5. Voice will be processed and sent as text

### Copy Messages
1. Hover over any message bubble
2. Click the copy icon that appears
3. Message content is copied to clipboard
4. "Copied!" feedback appears briefly

### Ask AI Help
1. Type your question in the input field
2. Click the ✨ (Ask AI) button instead of send
3. Get enhanced, detailed explanations from AI
4. Perfect for learning and understanding complex topics

### Resize Chat Window
1. Click the resize icon in the chat header
2. Toggle between small and large window sizes
3. Size preference persists during the session

## Customization

### Styling
Edit `book-frontend/src/components/ChatBot/styles.css` to customize:
- Colors and gradients
- Animations and transitions
- Responsive breakpoints
- Accessibility features

### AI Behavior
Modify the agent instructions in `book-backend/main.py`:
```python
agent = Agent(
    name="ChatBot Assistant", 
    instructions="Your custom instructions here...",
    model=model
)
```

### API Configuration
Update the API base URL in `book-frontend/src/components/ChatBot/api.ts`:
```typescript
const API_BASE_URL = 'http://your-backend-url:8000';
```

## Browser Support

- **Modern Browsers**: Chrome 90+, Firefox 88+, Safari 14+, Edge 90+
- **Voice Input**: Requires `MediaRecorder` API support
- **Clipboard**: Requires `navigator.clipboard` API support

## Performance Considerations

- **Session Storage**: Currently uses in-memory storage (implement database for production)
- **Message Limits**: No current limits (consider pagination for large conversations)
- **Voice Processing**: Mock implementation (integrate with speech recognition service)
- **Rate Limiting**: Not implemented (add for production use)

## Security Notes

- **CORS**: Currently allows all origins (restrict for production)
- **Input Validation**: Basic validation implemented (enhance for production)
- **API Keys**: Stored in environment variables (use secure key management)
- **Session Management**: Basic UUID-based sessions (implement proper auth)

## Troubleshooting

### Common Issues

1. **Chatbot not appearing:**
   - Check browser console for errors
   - Ensure backend is running on port 8000
   - Verify CORS settings

2. **Voice input not working:**
   - Check microphone permissions
   - Ensure HTTPS in production (required for microphone access)
   - Verify browser supports MediaRecorder API

3. **API connection errors:**
   - Verify backend server is running
   - Check API_BASE_URL in api.ts
   - Ensure GEMINI_API_KEY is set correctly

4. **Styling issues:**
   - Clear browser cache
   - Check for CSS conflicts with Docusaurus theme
   - Verify styles.css is being loaded

### Development Tips

- Use browser dev tools to inspect network requests
- Check console for React/TypeScript errors
- Monitor backend logs for API issues
- Test responsive design on different screen sizes

## Future Enhancements

- [ ] Database integration for persistent chat history
- [ ] Real speech recognition integration
- [ ] User authentication and personalization
- [ ] Message reactions and feedback
- [ ] File upload and sharing capabilities
- [ ] Multi-language support
- [ ] Chat export functionality
- [ ] Advanced AI model selection
- [ ] Real-time typing indicators
- [ ] Message search and filtering

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is part of the book-frontend/book-backend application. Please refer to the main project license.