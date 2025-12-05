import { ChatResponse, ChatHistoryResponse } from './types';

const API_BASE_URL = process.env.NODE_ENV === 'production' 
  ? (process.env.REACT_APP_API_URL || 'https://your-backend-url.com') 
  : 'http://localhost:8000';

class ChatAPI {
  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    const url = `${API_BASE_URL}${endpoint}`;
    
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