# Task List Refresh Event Contract

**Feature**: 007-frontend-chat-interface
**Date**: 2026-01-30
**Status**: Complete

## Overview

This document defines the event-driven contract for automatically refreshing the task list when the AI assistant modifies task data through chat interactions. This ensures the UI stays synchronized with backend state changes.

---

## Event Specification

### Event Name

**Name**: `tasks-updated`
**Type**: CustomEvent
**Scope**: Window-level event (global)
**Payload**: None (event carries no data)

---

## Event Dispatch (Producer)

### Trigger Conditions

The `tasks-updated` event MUST be dispatched when:
1. AI response is successfully received from backend
2. Response includes `tool_calls` array
3. `tool_calls` contains at least one task modification operation

**Task Modification Operations**:
- `add_task` - New task created
- `update_task` - Existing task modified
- `delete_task` - Task deleted
- `complete_task` - Task completion status changed

**Non-Triggering Operations**:
- `list_tasks` - Read-only query (no data modification)
- Empty `tool_calls` array - Conversational response only

### Dispatch Location

**Component**: `frontend-app/components/chat/chat-interface.tsx`
**Function**: `sendMessage()` or `useChat` hook
**Timing**: After AI response is received and displayed

### Implementation

```typescript
// In ChatInterface component or useChat hook
const sendMessage = async (content: string) => {
  try {
    // ... send message to backend ...
    const response = await apiClient.postChat(user.id, {
      message: content,
      conversation_id: conversationId
    });

    // ... update UI with response ...

    // Check if task data was modified
    const taskModificationTools = [
      'add_task',
      'update_task',
      'delete_task',
      'complete_task'
    ];

    const hasTaskModification = response.tool_calls.some(tool =>
      taskModificationTools.includes(tool)
    );

    if (hasTaskModification) {
      // Dispatch event to trigger task list refresh
      window.dispatchEvent(new CustomEvent('tasks-updated'));
    }

  } catch (error) {
    // Error handling...
  }
};
```

### Dispatch Guarantees

- ✅ Event dispatched only after successful AI response
- ✅ Event dispatched only once per chat response
- ✅ Event dispatched only for data-modifying operations
- ✅ Event dispatched after UI is updated with AI response
- ✅ Event carries no payload (listeners fetch fresh data)

---

## Event Listener (Consumer)

### Listener Location

**Component**: `frontend-app/app/dashboard/tasks/page.tsx`
**Hook**: `useEffect` with proper cleanup
**Timing**: Registered on component mount, cleaned up on unmount

### Implementation

```typescript
// In Task List component
function TaskListPage() {
  const [tasks, setTasks] = useState<Task[]>([]);
  const { user } = useAuth();

  // Fetch tasks function
  const fetchTasks = useCallback(async () => {
    try {
      const response = await taskService.getUserTasks(user.id);
      setTasks(response.tasks);
    } catch (error) {
      console.error('Failed to fetch tasks:', error);
    }
  }, [user.id]);

  // Initial fetch on mount
  useEffect(() => {
    fetchTasks();
  }, [fetchTasks]);

  // Listen for task updates from chat
  useEffect(() => {
    const handleTasksUpdated = () => {
      console.log('Tasks updated via chat, refreshing list...');
      fetchTasks();
    };

    window.addEventListener('tasks-updated', handleTasksUpdated);

    // Cleanup listener on unmount
    return () => {
      window.removeEventListener('tasks-updated', handleTasksUpdated);
    };
  }, [fetchTasks]);

  // ... rest of component ...
}
```

### Alternative: Custom Hook

For reusability, create a custom hook:

```typescript
// frontend-app/hooks/use-task-refresh.ts

/**
 * Custom hook to listen for task updates from chat
 * Automatically refreshes tasks when chat modifies data
 */
export function useTaskRefresh(onRefresh: () => void) {
  useEffect(() => {
    const handleTasksUpdated = () => {
      console.log('Tasks updated event received');
      onRefresh();
    };

    window.addEventListener('tasks-updated', handleTasksUpdated);

    return () => {
      window.removeEventListener('tasks-updated', handleTasksUpdated);
    };
  }, [onRefresh]);
}

// Usage in Task List component
function TaskListPage() {
  const fetchTasks = useCallback(async () => {
    // ... fetch logic ...
  }, [user.id]);

  // Listen for chat updates
  useTaskRefresh(fetchTasks);

  // ... rest of component ...
}
```

### Listener Guarantees

- ✅ Listener registered on component mount
- ✅ Listener cleaned up on component unmount
- ✅ No memory leaks from orphaned listeners
- ✅ Listener triggers fresh data fetch (no stale cache)
- ✅ Multiple listeners supported (e.g., stats component)

---

## Event Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│ User sends chat message: "Create a task to buy milk"       │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ ChatInterface sends request to backend                      │
│ POST /api/{user_id}/chat                                    │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ Backend processes message with AI agent                     │
│ Agent calls MCP tool: add_task                              │
│ Task created in database                                    │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ Backend returns response:                                   │
│ {                                                            │
│   conversation_id: 123,                                     │
│   response: "I've created a task...",                       │
│   tool_calls: ["add_task"]                                  │
│ }                                                            │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ ChatInterface receives response                             │
│ - Updates conversation_id                                   │
│ - Displays AI message in chat                               │
│ - Checks tool_calls for task modifications                  │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ ChatInterface dispatches event:                             │
│ window.dispatchEvent(new CustomEvent('tasks-updated'))      │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ TaskListPage receives event                                 │
│ - Event listener triggered                                  │
│ - Calls fetchTasks()                                        │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ TaskListPage fetches fresh data                             │
│ GET /api/{user_id}/tasks                                    │
│ - Receives updated task list including new task             │
│ - Updates UI with new data                                  │
└─────────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ User sees new task in list (< 1 second after chat response) │
└─────────────────────────────────────────────────────────────┘
```

---

## Edge Cases and Error Handling

### Edge Case 1: Multiple Rapid Updates

**Scenario**: User sends multiple task-modifying messages in quick succession

**Behavior**:
- Each response triggers separate `tasks-updated` event
- Task list may refresh multiple times
- Last refresh shows final state

**Mitigation** (Optional):
```typescript
// Debounce task refresh to avoid excessive API calls
const debouncedFetchTasks = useMemo(
  () => debounce(fetchTasks, 500),
  [fetchTasks]
);

useTaskRefresh(debouncedFetchTasks);
```

### Edge Case 2: Task List Not Mounted

**Scenario**: Chat is open but user is not on tasks page

**Behavior**:
- Event is dispatched but no listener is registered
- No error occurs (event is silently ignored)
- When user navigates to tasks page, fresh data is fetched on mount

**No Action Required**: This is expected behavior

### Edge Case 3: Listener Cleanup Failure

**Scenario**: Component unmounts but listener not cleaned up

**Behavior**:
- Memory leak (listener remains in memory)
- Listener may trigger on unmounted component (React warning)

**Prevention**:
```typescript
useEffect(() => {
  const handleTasksUpdated = () => {
    // Check if component is still mounted (optional)
    fetchTasks();
  };

  window.addEventListener('tasks-updated', handleTasksUpdated);

  // CRITICAL: Always return cleanup function
  return () => {
    window.removeEventListener('tasks-updated', handleTasksUpdated);
  };
}, [fetchTasks]);
```

### Edge Case 4: Fetch Fails After Event

**Scenario**: Event dispatched but task fetch fails (network error)

**Behavior**:
- Task list shows stale data
- Error message displayed to user
- User can manually refresh

**Handling**:
```typescript
const fetchTasks = useCallback(async () => {
  try {
    setLoading(true);
    const response = await taskService.getUserTasks(user.id);
    setTasks(response.tasks);
    setError(null);
  } catch (error) {
    console.error('Failed to fetch tasks:', error);
    setError('Failed to refresh tasks. Please try again.');
  } finally {
    setLoading(false);
  }
}, [user.id]);
```

---

## Testing

### Manual Testing Checklist

- [ ] **Task Creation**: Send "Create a task" in chat, verify task list refreshes automatically
- [ ] **Task Update**: Send "Update task title" in chat, verify task list shows updated title
- [ ] **Task Deletion**: Send "Delete task" in chat, verify task disappears from list
- [ ] **Task Completion**: Send "Mark task as complete" in chat, verify completion status updates
- [ ] **Read-Only Query**: Send "What tasks do I have?" in chat, verify task list does NOT refresh
- [ ] **Multiple Updates**: Send multiple task-modifying messages rapidly, verify all updates reflected
- [ ] **Component Unmount**: Open chat, modify task, navigate away, verify no errors
- [ ] **Network Error**: Disconnect network, send task-modifying message, verify error handling

### Automated Testing (Optional)

```typescript
// Test event dispatch
describe('ChatInterface', () => {
  it('should dispatch tasks-updated event when AI modifies tasks', async () => {
    const eventSpy = jest.fn();
    window.addEventListener('tasks-updated', eventSpy);

    // Mock API response with task modification
    mockApiClient.postChat.mockResolvedValue({
      conversation_id: 123,
      response: 'Task created',
      tool_calls: ['add_task']
    });

    // Send message
    await sendMessage('Create a task');

    // Verify event was dispatched
    expect(eventSpy).toHaveBeenCalledTimes(1);

    window.removeEventListener('tasks-updated', eventSpy);
  });

  it('should NOT dispatch event for read-only queries', async () => {
    const eventSpy = jest.fn();
    window.addEventListener('tasks-updated', eventSpy);

    // Mock API response with read-only query
    mockApiClient.postChat.mockResolvedValue({
      conversation_id: 123,
      response: 'You have 3 tasks',
      tool_calls: ['list_tasks']
    });

    // Send message
    await sendMessage('What tasks do I have?');

    // Verify event was NOT dispatched
    expect(eventSpy).not.toHaveBeenCalled();

    window.removeEventListener('tasks-updated', eventSpy);
  });
});

// Test event listener
describe('TaskListPage', () => {
  it('should refresh tasks when tasks-updated event is dispatched', async () => {
    const { rerender } = render(<TaskListPage />);

    // Initial tasks
    expect(screen.getByText('Task 1')).toBeInTheDocument();

    // Mock updated tasks
    mockTaskService.getUserTasks.mockResolvedValue({
      tasks: [
        { id: '1', title: 'Task 1' },
        { id: '2', title: 'New Task' }
      ]
    });

    // Dispatch event
    window.dispatchEvent(new CustomEvent('tasks-updated'));

    // Wait for refresh
    await waitFor(() => {
      expect(screen.getByText('New Task')).toBeInTheDocument();
    });
  });
});
```

---

## Performance Considerations

### Event Dispatch Performance

**Impact**: Negligible
- CustomEvent creation: < 1ms
- Event dispatch: < 1ms
- Total overhead: < 2ms per chat response

### Task Refresh Performance

**Impact**: Depends on task count and network latency
- API request: 50-200ms (typical)
- Rendering: 10-50ms (for 10-100 tasks)
- Total: < 250ms (meets < 1 second requirement)

**Optimization** (if needed):
- Debounce rapid updates (500ms delay)
- Use React Query for automatic caching
- Implement optimistic updates in chat

---

## Alternative Approaches (Not Chosen)

### Alternative 1: Callback Props

```typescript
// Dashboard Layout
<ChatInterface onTasksModified={() => taskListRef.current?.refresh()} />
```

**Pros**: Type-safe, explicit dependency
**Cons**: Requires prop drilling, tight coupling, harder to extend

**Why Not Chosen**: Violates loose coupling principle, harder to maintain

### Alternative 2: Context API

```typescript
// Create context
const TaskRefreshContext = createContext<() => void>(() => {});

// Provider in layout
<TaskRefreshContext.Provider value={refreshTasks}>
  <ChatInterface />
  <TaskList />
</TaskRefreshContext.Provider>
```

**Pros**: Type-safe, no prop drilling
**Cons**: Overkill for simple refresh, adds complexity, not used elsewhere

**Why Not Chosen**: Too complex for simple use case, inconsistent with codebase patterns

### Alternative 3: React Query / SWR

```typescript
// Automatic cache invalidation
const { data, mutate } = useSWR('/api/tasks', fetcher);

// In chat
mutate(); // Triggers refetch
```

**Pros**: Automatic caching, built-in refetching
**Cons**: Requires new dependency, significant refactoring

**Why Not Chosen**: Not used elsewhere in codebase, too heavyweight

---

## Contract Guarantees

### Producer (ChatInterface) Guarantees
- ✅ Event dispatched only for task-modifying operations
- ✅ Event dispatched after successful AI response
- ✅ Event dispatched only once per response
- ✅ Event carries no payload (stateless)
- ✅ Event name is consistent (`tasks-updated`)

### Consumer (TaskList) Guarantees
- ✅ Listener registered on component mount
- ✅ Listener cleaned up on component unmount
- ✅ Fresh data fetched on event (no stale cache)
- ✅ Error handling for failed fetches
- ✅ No memory leaks from orphaned listeners

### System Guarantees
- ✅ Loose coupling between chat and task list
- ✅ Multiple listeners supported (extensible)
- ✅ No race conditions (event-driven, not polling)
- ✅ Performance impact < 250ms (meets < 1 second requirement)

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01-30 | Initial contract definition |

---

## Related Documents

- Chat API Contract: `specs/007-frontend-chat-interface/contracts/chat-api.md`
- Data Models: `specs/007-frontend-chat-interface/data-model.md`
- Implementation Plan: `specs/007-frontend-chat-interface/plan.md`
