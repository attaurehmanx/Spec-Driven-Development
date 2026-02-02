# Data Model: Frontend Chat Interface

**Feature**: 007-frontend-chat-interface
**Date**: 2026-01-30
**Status**: Complete

## Overview

This document defines the data models and TypeScript interfaces for the Frontend Chat Interface feature. All models are frontend-only and represent UI state, not database entities.

---

## Core Data Models

### ChatMessage

Represents a single message in the conversation (user or AI).

```typescript
interface ChatMessage {
  /**
   * Unique identifier for the message (client-side)
   * Used for React keys and optimistic updates
   */
  id: string;

  /**
   * Message sender role
   * - 'user': Message sent by the authenticated user
   * - 'assistant': Message sent by the AI assistant
   */
  role: 'user' | 'assistant';

  /**
   * Message content (text)
   * Max length: 10,000 characters (enforced by backend)
   */
  content: string;

  /**
   * Timestamp when message was created
   * Used for display and sorting
   */
  timestamp: Date;

  /**
   * Message status (optional, for optimistic updates)
   * - 'sending': User message being sent to backend
   * - 'sent': Message successfully sent and response received
   * - 'error': Message failed to send
   * - undefined: AI message (no status needed)
   */
  status?: 'sending' | 'sent' | 'error';

  /**
   * Error message if status is 'error' (optional)
   */
  errorMessage?: string;
}
```

**Usage Example**:
```typescript
const userMessage: ChatMessage = {
  id: crypto.randomUUID(),
  role: 'user',
  content: 'Create a task to buy milk',
  timestamp: new Date(),
  status: 'sending'
};

const aiMessage: ChatMessage = {
  id: crypto.randomUUID(),
  role: 'assistant',
  content: 'I\'ve created a task titled "Buy milk" for you.',
  timestamp: new Date()
};
```

---

### ConversationState

Represents the complete state of the chat interface.

```typescript
interface ConversationState {
  /**
   * Backend conversation ID
   * - null: New conversation (not yet created on backend)
   * - number: Existing conversation ID from backend
   */
  conversationId: number | null;

  /**
   * Array of all messages in the conversation
   * Ordered chronologically (oldest first)
   */
  messages: ChatMessage[];

  /**
   * Loading state indicator
   * - true: Waiting for AI response
   * - false: Ready to send new message
   */
  isLoading: boolean;

  /**
   * Error message if last operation failed
   * - null: No error
   * - string: Error message to display to user
   */
  error: string | null;
}
```

**Initial State**:
```typescript
const initialState: ConversationState = {
  conversationId: null,
  messages: [],
  isLoading: false,
  error: null
};
```

---

## API Request/Response Models

### ChatRequest

Request payload for `POST /api/{user_id}/chat` endpoint.

```typescript
interface ChatRequest {
  /**
   * User's message text
   * Constraints:
   * - Min length: 1 character (after trimming)
   * - Max length: 10,000 characters
   * - Cannot be empty or whitespace-only
   */
  message: string;

  /**
   * Conversation ID for multi-turn conversations
   * - null or undefined: Start new conversation
   * - number: Continue existing conversation
   */
  conversation_id?: number | null;
}
```

**Example**:
```typescript
// New conversation
const newConversationRequest: ChatRequest = {
  message: 'What tasks do I have today?',
  conversation_id: null
};

// Continue existing conversation
const followUpRequest: ChatRequest = {
  message: 'Mark the first one as complete',
  conversation_id: 123
};
```

---

### ChatResponse

Response payload from `POST /api/{user_id}/chat` endpoint.

```typescript
interface ChatResponse {
  /**
   * Conversation ID
   * - For new conversations: newly created ID
   * - For existing conversations: same ID as request
   */
  conversation_id: number;

  /**
   * AI assistant's response text
   */
  response: string;

  /**
   * List of MCP tools called by the AI
   * Used to trigger task list refresh
   *
   * Possible values:
   * - 'add_task': Task was created
   * - 'update_task': Task was modified
   * - 'delete_task': Task was deleted
   * - 'complete_task': Task completion status changed
   * - 'list_tasks': Tasks were queried (no refresh needed)
   */
  tool_calls: string[];
}
```

**Example**:
```typescript
const response: ChatResponse = {
  conversation_id: 123,
  response: 'I\'ve created a task titled "Buy milk" for you.',
  tool_calls: ['add_task']
};
```

---

## Hook Return Types

### UseChatReturn

Return type for the `useChat` custom hook.

```typescript
interface UseChatReturn {
  /**
   * Current conversation state
   */
  conversationId: number | null;
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;

  /**
   * Send a message to the AI assistant
   * @param content - Message text to send
   * @returns Promise that resolves when response is received
   */
  sendMessage: (content: string) => Promise<void>;

  /**
   * Start a new conversation
   * Clears messages and resets conversation_id
   */
  startNewConversation: () => void;

  /**
   * Clear error message
   */
  clearError: () => void;
}
```

**Usage Example**:
```typescript
function ChatInterface() {
  const {
    messages,
    isLoading,
    error,
    sendMessage,
    startNewConversation,
    clearError
  } = useChat();

  // Component implementation...
}
```

---

## Component Props

### ChatInterfaceProps

Props for the main ChatInterface component.

```typescript
interface ChatInterfaceProps {
  /**
   * Whether the chat interface is open
   */
  isOpen: boolean;

  /**
   * Callback to close the chat interface
   */
  onClose: () => void;

  /**
   * Optional callback when tasks are modified
   * Alternative to event-driven refresh
   */
  onTasksModified?: () => void;

  /**
   * Optional CSS class name for styling
   */
  className?: string;
}
```

---

### ChatMessageProps

Props for individual message components.

```typescript
interface ChatMessageProps {
  /**
   * Message data to display
   */
  message: ChatMessage;

  /**
   * Whether this is the most recent message
   * Used for scroll-to-bottom behavior
   */
  isLatest?: boolean;
}
```

---

### ChatInputProps

Props for the message input component.

```typescript
interface ChatInputProps {
  /**
   * Callback when user submits a message
   * @param content - Message text entered by user
   */
  onSendMessage: (content: string) => void;

  /**
   * Whether the input should be disabled
   * Typically true when isLoading is true
   */
  disabled: boolean;

  /**
   * Placeholder text for the input
   */
  placeholder?: string;
}
```

---

## Validation Rules

### Message Content Validation

```typescript
/**
 * Validates message content before sending
 * @param content - Message text to validate
 * @returns Validation result with error message if invalid
 */
function validateMessageContent(content: string): {
  isValid: boolean;
  error?: string;
} {
  // Trim whitespace
  const trimmed = content.trim();

  // Check if empty
  if (trimmed.length === 0) {
    return {
      isValid: false,
      error: 'Message cannot be empty'
    };
  }

  // Check max length
  if (trimmed.length > 10000) {
    return {
      isValid: false,
      error: 'Message is too long (max 10,000 characters)'
    };
  }

  return { isValid: true };
}
```

---

## State Transitions

### Message Status Flow

```
User sends message:
  null → 'sending' → 'sent' (success)
                  → 'error' (failure)

AI response:
  No status (AI messages don't have status)
```

### Conversation ID Flow

```
New conversation:
  conversationId: null
  ↓ (send first message)
  conversationId: 123 (from backend response)
  ↓ (send follow-up messages)
  conversationId: 123 (unchanged)

Start new conversation:
  conversationId: 123
  ↓ (user clicks "New Conversation")
  conversationId: null
  messages: []
```

---

## Error Handling

### Error Types

```typescript
type ChatError =
  | 'network_error'      // Network request failed
  | 'auth_error'         // Authentication failed (401)
  | 'validation_error'   // Message validation failed
  | 'server_error'       // Backend error (500)
  | 'unknown_error';     // Unexpected error

interface ChatErrorDetails {
  type: ChatError;
  message: string;
  originalError?: Error;
}
```

### Error Messages

User-friendly error messages for each error type:

```typescript
const ERROR_MESSAGES: Record<ChatError, string> = {
  network_error: 'Unable to connect to the server. Please check your internet connection.',
  auth_error: 'Your session has expired. Please sign in again.',
  validation_error: 'Invalid message. Please check your input.',
  server_error: 'The server encountered an error. Please try again later.',
  unknown_error: 'An unexpected error occurred. Please try again.'
};
```

---

## Type Guards

### Type guard functions for runtime type checking

```typescript
/**
 * Checks if a value is a valid ChatMessage
 */
function isChatMessage(value: unknown): value is ChatMessage {
  return (
    typeof value === 'object' &&
    value !== null &&
    'id' in value &&
    'role' in value &&
    'content' in value &&
    'timestamp' in value &&
    (value.role === 'user' || value.role === 'assistant')
  );
}

/**
 * Checks if a value is a valid ChatResponse
 */
function isChatResponse(value: unknown): value is ChatResponse {
  return (
    typeof value === 'object' &&
    value !== null &&
    'conversation_id' in value &&
    'response' in value &&
    'tool_calls' in value &&
    typeof value.conversation_id === 'number' &&
    typeof value.response === 'string' &&
    Array.isArray(value.tool_calls)
  );
}
```

---

## Persistence Strategy

### LocalStorage Schema (Optional Enhancement)

If implementing conversation_id persistence:

```typescript
interface ChatStorageData {
  conversationId: number | null;
  lastUpdated: string; // ISO date string
}

const STORAGE_KEY = 'chat_conversation_id';

// Save to localStorage
function saveConversationId(conversationId: number | null): void {
  const data: ChatStorageData = {
    conversationId,
    lastUpdated: new Date().toISOString()
  };
  localStorage.setItem(STORAGE_KEY, JSON.stringify(data));
}

// Load from localStorage
function loadConversationId(): number | null {
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (!stored) return null;

    const data: ChatStorageData = JSON.parse(stored);

    // Optional: Expire after 24 hours
    const lastUpdated = new Date(data.lastUpdated);
    const now = new Date();
    const hoursSinceUpdate = (now.getTime() - lastUpdated.getTime()) / (1000 * 60 * 60);

    if (hoursSinceUpdate > 24) {
      localStorage.removeItem(STORAGE_KEY);
      return null;
    }

    return data.conversationId;
  } catch (error) {
    console.error('Failed to load conversation ID:', error);
    return null;
  }
}
```

---

## Summary

### Key Data Models
- **ChatMessage**: Individual message with role, content, timestamp, and status
- **ConversationState**: Complete chat state with messages, conversation_id, loading, and error
- **ChatRequest/Response**: API contract types matching backend endpoint
- **UseChatReturn**: Hook interface for state management

### Design Principles
- **Type Safety**: All models have strict TypeScript types
- **Immutability**: State updates create new objects (React best practice)
- **Validation**: Input validation before API calls
- **Error Handling**: Comprehensive error types and user-friendly messages
- **Optimistic Updates**: User messages shown immediately with status tracking

### Next Steps
- Implement these types in `frontend-app/types.ts`
- Use these models in `useChat` hook implementation
- Reference these types in all chat components
- Ensure API client returns properly typed responses
