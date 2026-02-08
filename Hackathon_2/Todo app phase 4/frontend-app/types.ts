// Task-related types
export interface Task {
  id: string; // UUID
  title: string;
  description?: string;
  completed: boolean;
  owner_id: string; // UUID
  created_at: string; // ISO date string
  updated_at: string; // ISO date string
}

export interface TaskFormData {
  title: string;
  description?: string;
  completed: boolean;
}

export interface TaskListResponse {
  tasks: Task[];
}

export interface TaskResponse {
  id: string; // UUID
  title: string;
  description?: string;
  completed: boolean;
  owner_id: string; // UUID
  created_at: string; // ISO date string
  updated_at: string; // ISO date string
}

export interface ErrorResponse {
  error: string;
  message?: string;
}

// Auth-related types
export interface UserSession {
  id: string;
  email: string;
  name?: string;
  first_name?: string;
  last_name?: string;
  createdAt: string;
  updatedAt?: string;
  phone?: string;
  address?: string;
  avatar?: string;
}

export interface AuthUIState {
  isAuthenticated: boolean;
  user: UserSession | null;
  loading: boolean;
  error: string | null;
}

// Chat-related types
export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  status?: 'sending' | 'sent' | 'error';
  errorMessage?: string;
}

export interface ConversationState {
  conversationId: number | null;
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;
}

export interface ChatRequest {
  message: string;
  conversation_id?: number | null;
}

export interface ChatResponse {
  conversation_id: number;
  response: string;
  tool_calls: string[];
}

export interface UseChatReturn {
  conversationId: number | null;
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;
  sendMessage: (content: string) => Promise<void>;
  startNewConversation: () => void;
  clearError: () => void;
}