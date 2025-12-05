export interface ChatMessage {
  id: string;
  content: string;
  sender: "user" | "ai";
  timestamp: Date;
  sessionId: string;
}

export interface ChatSession {
  id: string;
  messages: ChatMessage[];
  createdAt: Date;
  updatedAt: Date;
}

export interface ChatResponse {
  message: ChatMessage;
  sessionId: string;
}

export interface ChatHistoryResponse {
  messages: ChatMessage[];
}

export interface VoiceInputProps {
  onVoiceInput: (audioBlob: Blob) => void;
  isRecording: boolean;
  onToggleRecording: () => void;
}

export interface MessageProps {
  message: ChatMessage;
  onCopy: (text: string) => void;
}

export interface ChatWindowProps {
  messages: ChatMessage[];
  onSendMessage: (content: string, isAskAI?: boolean) => void;
  onVoiceInput: (audioBlob: Blob) => void;
  onFileUpload: (file: File) => void;
  onCopyMessage: (text: string) => void;
  onClose: () => void;
  onToggleSize: () => void;
  onShowHistory: () => void;
  windowSize: "small" | "large";
  isLoading: boolean;
}
