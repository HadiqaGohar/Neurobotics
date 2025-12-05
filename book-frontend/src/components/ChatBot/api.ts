import { ChatResponse, ChatHistoryResponse } from './types';

const API_BASE_URL = process.env.NODE_ENV === 'production' 
  ? (process.env.REACT_APP_API_URL || '') 
  : 'http://localhost:8000';

class ChatAPI {
  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    // If no API URL is configured, return mock response
    if (!API_BASE_URL) {
      return this.getMockResponse<T>(endpoint, options);
    }

    const url = `${API_BASE_URL}${endpoint}`;
    
    try {
      const response = await fetch(url, {
        headers: {
          'Content-Type': 'application/json',
          ...options.headers,
        },
        ...options,
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return response.json();
    } catch (error) {
      console.warn('API request failed, using mock response:', error);
      return this.getMockResponse<T>(endpoint, options);
    }
  }

  private getMockResponse<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    // Mock responses for demo purposes
    const mockResponses: Record<string, any> = {
      '/chat': {
        message: {
          id: `ai_${Date.now()}`,
          content: "Hello! I'm a demo chatbot. In production, I would be connected to a real AI backend. You can still interact with me to see the UI features!",
          sender: 'ai',
          timestamp: new Date(),
          sessionId: 'demo_session'
        }
      },
      '/chat/ask-ai': {
        message: {
          id: `ai_${Date.now()}`,
          content: "This is the Ask AI feature! In a real deployment, this would provide enhanced AI responses with more detailed explanations and context.",
          sender: 'ai',
          timestamp: new Date(),
          sessionId: 'demo_session'
        }
      },
      '/chat/voice': {
        message: {
          id: `ai_${Date.now()}`,
          content: "Voice input received! In production, this would process your speech and respond accordingly.",
          sender: 'ai',
          timestamp: new Date(),
          sessionId: 'demo_session'
        }
      },
      '/chat/file': {
        message: {
          id: `ai_${Date.now()}`,
          content: "File uploaded successfully! In production, I would analyze your file and provide relevant insights.",
          sender: 'ai',
          timestamp: new Date(),
          sessionId: 'demo_session'
        }
      }
    };

    const baseEndpoint = endpoint.split('?')[0];
    const mockResponse = mockResponses[baseEndpoint] || {
      message: {
        id: `ai_${Date.now()}`,
        content: "This is a demo response. Connect a real backend to enable full functionality!",
        sender: 'ai',
        timestamp: new Date(),
        sessionId: 'demo_session'
      }
    };

    return Promise.resolve(mockResponse as T);
  }

  async sendMessage(message: string, sessionId?: string): Promise<ChatResponse> {
    return this.request<ChatResponse>('/chat', {
      method: 'POST',
      body: JSON.stringify({
        message,
        session_id: sessionId,
      }),
    });
  }

  async getChatHistory(sessionId: string): Promise<ChatHistoryResponse> {
    if (!API_BASE_URL) {
      return Promise.resolve({ messages: [] } as ChatHistoryResponse);
    }
    return this.request<ChatHistoryResponse>(`/chat/history/${sessionId}`);
  }

  async processVoice(audioData: string, sessionId: string): Promise<ChatResponse> {
    return this.request<ChatResponse>('/chat/voice', {
      method: 'POST',
      body: JSON.stringify({
        audio_data: audioData,
        session_id: sessionId,
      }),
    });
  }

  async askAI(message: string, sessionId?: string): Promise<ChatResponse> {
    return this.request<ChatResponse>('/chat/ask-ai', {
      method: 'POST',
      body: JSON.stringify({
        message,
        session_id: sessionId,
      }),
    });
  }

  async processFile(file: File, sessionId: string): Promise<ChatResponse> {
    const formData = new FormData();
    formData.append('file', file);
    formData.append('session_id', sessionId);

    const response = await fetch(`${API_BASE_URL}/chat/file`, {
      method: 'POST',
      body: formData,
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return response.json();
  }
}

export const chatAPI = new ChatAPI();