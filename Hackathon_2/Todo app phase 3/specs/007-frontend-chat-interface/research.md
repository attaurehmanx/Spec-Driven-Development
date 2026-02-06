# Research: Frontend Chat Interface

**Feature**: 007-frontend-chat-interface
**Date**: 2026-01-30
**Status**: Complete

## Overview

This document captures research findings and technology decisions for implementing the Frontend Chat Interface feature. All "NEEDS CLARIFICATION" items from the Technical Context have been resolved.

---

## R1: Chat UI Library Evaluation

### Question
Should we use OpenAI ChatKit, build custom components, or use an alternative library?

### Investigation

**OpenAI ChatKit Analysis**:
- **Compatibility**: Requires investigation of Next.js 16+ and React 19 compatibility
- **Bundle Size**: Unknown without testing
- **Customization**: Likely limited to predefined themes
- **Documentation**: Official OpenAI documentation available
- **Risk**: Potential compatibility issues with App Router and Server Components

**Alternative Libraries**:
- `react-chat-elements`: Mature, but may have styling conflicts with Tailwind
- `stream-chat-react`: Feature-rich but heavyweight (designed for Stream Chat service)
- `@chatscope/chat-ui-kit-react`: Good TypeScript support, but additional dependency

**Custom Components Analysis**:
- **Pros**: Full control, matches existing design system, minimal bundle size, no external dependencies
- **Cons**: More implementation work, need to handle edge cases ourselves
- **Existing Assets**: We already have Button, Card, LoadingSpinner, and Tailwind CSS

### Decision

**✅ Build Custom Components**

**Rationale**:
1. **Design System Alignment**: Existing UI components (Button, Card, LoadingSpinner) already match our Tailwind design system
2. **Bundle Size**: No additional dependencies means smaller bundle
3. **Flexibility**: Full control over behavior and styling
4. **Simplicity**: Chat interface requirements are straightforward (message list, input, bubbles)
5. **Risk Mitigation**: No compatibility concerns with Next.js 16 or React 19
6. **Existing Patterns**: Can follow patterns from existing dashboard and task components

**Implementation Approach**:
- Use existing UI components as building blocks
- Create 6 focused chat components (interface, message-list, input, message, header, loading)
- Style with Tailwind CSS utilities matching existing design
- Leverage existing patterns from `use-auth.ts` for state management

---

## R2: Conversation State Management Pattern

### Question
How should we manage conversation state (messages, conversation_id, loading)?

### Investigation

**Existing Patterns in Codebase**:
- `use-auth.ts`: Uses useState for authentication state (isAuthenticated, user, error, isLoading)
- Task list page: Uses useState for tasks array, loading, error, filter
- No global state management library (Redux, Zustand, etc.)
- Pattern: Local component state with custom hooks

**Options Evaluated**:
1. **useState**: Simple, follows existing patterns, good for independent state
2. **useReducer**: Better for complex state transitions, more boilerplate
3. **Context API**: Overkill for component-local state, adds complexity
4. **localStorage**: Good for conversation_id persistence, not for full state

### Decision

**✅ Custom Hook with useState**

**Rationale**:
1. **Consistency**: Matches existing `use-auth.ts` pattern
2. **Simplicity**: Chat state is not complex enough to warrant useReducer
3. **Encapsulation**: Hook encapsulates all chat logic in one place
4. **Testability**: Easy to test hook independently
5. **Performance**: useState is sufficient for our use case

**Implementation Pattern**:
```typescript
// hooks/use-chat.ts
export function useChat() {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [conversationId, setConversationId] = useState<number | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const sendMessage = async (content: string) => {
    // Implementation
  };

  const startNewConversation = () => {
    setMessages([]);
    setConversationId(null);
  };

  return { messages, conversationId, isLoading, error, sendMessage, startNewConversation };
}
```

**Optional Enhancement**: Store conversation_id in localStorage for persistence across page refreshes (implement if time permits).

---

## R3: Task List Refresh Mechanism

### Question
What's the best way to trigger task list refresh when AI modifies data?

### Investigation

**Options Evaluated**:

**Option A: Custom Event (window.dispatchEvent / addEventListener)**
- ✅ Loose coupling between chat and task list
- ✅ No prop drilling required
- ✅ Works across component boundaries
- ✅ Easy to add more listeners if needed
- ⚠️ Requires proper cleanup to avoid memory leaks
- ⚠️ Less type-safe than other options

**Option B: Callback Prop**
- ✅ Type-safe
- ✅ Explicit dependency
- ❌ Requires prop drilling from dashboard layout to chat component
- ❌ Tight coupling between components
- ❌ Harder to extend to other components

**Option C: Context API**
- ✅ Type-safe
- ✅ No prop drilling
- ❌ Overkill for simple refresh trigger
- ❌ Adds complexity with Provider/Consumer
- ❌ Not used elsewhere in codebase

**Option D: React Query / SWR**
- ✅ Automatic cache invalidation
- ✅ Built-in refetching
- ❌ Requires adding new dependency
- ❌ Significant refactoring of existing task list
- ❌ Not used elsewhere in codebase

### Decision

**✅ Custom Event Pattern (Option A)**

**Rationale**:
1. **Loose Coupling**: Chat and task list remain independent
2. **Simplicity**: Minimal code changes required
3. **Extensibility**: Easy to add more listeners (e.g., stats refresh)
4. **No Dependencies**: Uses native browser APIs
5. **Proven Pattern**: Commonly used for cross-component communication

**Implementation**:

**Dispatch (in ChatInterface)**:
```typescript
// After receiving AI response
if (response.tool_calls.some(tool =>
  ['add_task', 'update_task', 'delete_task', 'complete_task'].includes(tool)
)) {
  window.dispatchEvent(new CustomEvent('tasks-updated'));
}
```

**Listen (in Task List)**:
```typescript
useEffect(() => {
  const handleTasksUpdated = () => {
    fetchTasks(); // Refresh task list
  };

  window.addEventListener('tasks-updated', handleTasksUpdated);
  return () => window.removeEventListener('tasks-updated', handleTasksUpdated);
}, [fetchTasks]);
```

**Safety Measures**:
- Proper cleanup in useEffect return function
- Event dispatched only after successful AI response
- No payload in event (task list fetches fresh data)
- Debouncing if multiple rapid updates occur (optional)

---

## R4: Missing Service Modules

### Question
How should we implement the missing token-storage and task-service modules?

### Investigation

**Current References**:
- `api-client.js` expects: `tokenStorage.getAccessToken()`, `tokenStorage.isTokenExpired()`, `tokenStorage.verifyAndValidateToken()`
- Dashboard pages expect: `taskService.getUserTasks()`, `taskService.getTaskStats()`
- `use-auth.ts` expects: Token verification and refresh functionality

**Token Storage Requirements**:
```typescript
interface TokenStorage {
  getAccessToken(): string | null;
  setAccessToken(token: string): void;
  getRefreshToken(): string | null;
  setRefreshToken(token: string): void;
  clearTokens(): void;
  isTokenExpired(token: string): boolean;
  verifyAndValidateToken(): Promise<UserSession>;
}
```

**Task Service Requirements**:
```typescript
interface TaskService {
  getUserTasks(userId: string): Promise<{ tasks: Task[] }>;
  getTaskStats(userId: string): Promise<{ total: number, completed: number, pending: number }>;
  createTask(userId: string, task: Partial<Task>): Promise<Task>;
  updateTask(userId: string, taskId: string, updates: Partial<Task>): Promise<Task>;
  deleteTask(userId: string, taskId: string): Promise<void>;
  toggleTaskCompletion(userId: string, taskId: string): Promise<Task>;
}
```

### Decision

**✅ Implement Minimal Required Functionality**

**Rationale**:
1. **Unblock Development**: Chat interface needs working authentication
2. **Follow Existing Patterns**: Wrap existing API client methods
3. **Security Best Practices**: Use localStorage for tokens (httpOnly cookies ideal but requires backend changes)
4. **Focused Scope**: Only implement what's needed for chat interface

**Implementation Priority**:

**Phase 1 (Critical for Chat)**:
- `token-storage.ts`: Token management with localStorage
- `utils.ts`: `cn()` function for className merging (required by UI components)

**Phase 2 (Required for Dashboard)**:
- `task-service.ts`: Wrapper around api-client task methods

**Token Storage Strategy**:
- Store access token in localStorage (key: `access_token`)
- Store refresh token in localStorage (key: `refresh_token`)
- JWT expiration check using `jwt-decode` or manual parsing
- Token refresh flow using existing API client refresh method

---

## Additional Findings

### F1: Existing API Client Patterns

**Discovery**: The `api-client.js` already has comprehensive error handling:
- Automatic token refresh on 401 errors
- Event dispatch on auth failure (`auth-token-invalidated`)
- Retry logic for failed requests
- Proper error propagation

**Impact**: Chat API integration can leverage existing error handling. No need to duplicate logic.

### F2: Existing UI Component Library

**Discovery**: Comprehensive UI component library exists in `components/ui/`:
- Button with variants and loading states
- Card for message containers
- LoadingSpinner for typing indicators
- Badge for status indicators
- Avatar for user/AI icons

**Impact**: Can build chat interface entirely with existing components. No new UI primitives needed.

### F3: TypeScript Configuration

**Discovery**: Project uses TypeScript with strict mode enabled
- Path aliases configured (`@/*` maps to root)
- Strict null checks enabled
- All components use `.tsx` extension

**Impact**: All new code must follow strict TypeScript patterns. Type safety is enforced.

### F4: Responsive Design Patterns

**Discovery**: Existing components use Tailwind responsive utilities:
- Mobile-first approach with `sm:`, `md:`, `lg:` breakpoints
- Sidebar collapses on mobile in dashboard layout
- Cards stack vertically on small screens

**Impact**: Chat interface must follow same responsive patterns. Sidebar on desktop, modal on mobile.

---

## Technology Stack Summary

### Confirmed Technologies
- **Frontend Framework**: Next.js 16.1.1 with App Router
- **Language**: TypeScript 5.x
- **Styling**: Tailwind CSS 4.x
- **UI Components**: Custom components in `components/ui/`
- **State Management**: React hooks (useState, useEffect, custom hooks)
- **API Client**: Existing `services/api-client.js`
- **Authentication**: JWT tokens via Better Auth

### No Additional Dependencies Required
- ✅ No chat UI library needed
- ✅ No state management library needed
- ✅ No additional HTTP client needed
- ✅ No CSS framework needed

### Optional Dependencies (if needed)
- `jwt-decode`: For token expiration checking (lightweight, 2KB)
- `uuid`: For client-side message IDs (already may be available)

---

## Risk Mitigation

### Identified Risks and Mitigations

**Risk**: Event listener memory leaks
- **Mitigation**: Proper cleanup in useEffect return function
- **Testing**: Mount/unmount components multiple times

**Risk**: Race conditions in task refresh
- **Mitigation**: Event dispatched only after AI response received
- **Testing**: Rapid successive task operations

**Risk**: Token expiration during chat session
- **Mitigation**: Leverage existing API client token refresh
- **Testing**: Test with expired tokens

**Risk**: Long conversation performance
- **Mitigation**: Start with simple array, add virtualization if needed
- **Testing**: Test with 100+ messages

---

## Recommendations

### Implementation Order
1. **Foundation First**: Implement token-storage and utils before chat components
2. **API Integration Second**: Add postChat to api-client and test independently
3. **State Management Third**: Create use-chat hook with proper TypeScript types
4. **UI Components Fourth**: Build chat components using existing UI library
5. **Integration Fifth**: Add to dashboard layout and wire up refresh mechanism
6. **Polish Last**: Error handling, loading states, responsive design

### Testing Strategy
- **Unit Testing**: Test use-chat hook independently
- **Integration Testing**: Test chat + task refresh flow
- **Manual Testing**: Test on multiple screen sizes and browsers
- **Error Testing**: Test with network failures, expired tokens, backend errors

### Performance Considerations
- **Lazy Loading**: Consider lazy loading chat component (React.lazy)
- **Message Virtualization**: Only if conversations exceed 100 messages
- **Debouncing**: Debounce task refresh if multiple rapid updates
- **Memoization**: Use React.memo for message components if performance issues

---

## Conclusion

All research questions have been resolved with clear decisions:
- ✅ **Chat UI**: Build custom components using existing UI library
- ✅ **State Management**: Custom hook with useState pattern
- ✅ **Task Refresh**: Custom event pattern for loose coupling
- ✅ **Missing Services**: Implement minimal required functionality

**Next Step**: Proceed to Phase 1 (Design & Contracts) to create data models and API contracts.

**Confidence Level**: High - All decisions are based on existing codebase patterns and proven approaches.
