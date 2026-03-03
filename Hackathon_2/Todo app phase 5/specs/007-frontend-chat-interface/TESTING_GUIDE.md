# Frontend Chat Interface - Manual Testing Guide

**Date**: 2026-01-30
**Tester**: [Your Name]
**Feature**: 007-frontend-chat-interface
**Status**: Ready for Testing

## Pre-Testing Setup

### 1. Verify Services Are Running

**Backend Service**:
```bash
cd "Z:\phse 33\backend"
# Activate virtual environment
venv\Scripts\activate
# Start backend
uvicorn main:app --reload --port 8000
```

**Verify Backend**:
- Open browser to: http://localhost:8000/docs
- Should see FastAPI Swagger documentation
- Or use curl: `curl http://localhost:8000/health`
- Expected: `{"status": "ok"}`

**Frontend Service**:
```bash
cd "Z:\phse 33\frontend-app"
npm run dev
```

**Verify Frontend**:
- Open browser to: http://localhost:3000
- Should see application homepage
- Check browser console for errors (F12 → Console)

### 2. Sign In with Test User

1. Navigate to: http://localhost:3000/sign-in
2. Sign in with your test account
3. Should redirect to: http://localhost:3000/dashboard
4. Verify you see the task list page

### 3. Open Browser DevTools

Press F12 to open DevTools and keep these tabs visible:
- **Console**: Monitor for errors and logs
- **Network**: Monitor API requests
- **Application**: Check localStorage for tokens

---

## Test Suite

### Test 1: Basic Chat Interaction (User Story 1)

**Objective**: Verify user can open chat, send messages, and receive AI responses

#### Test 1.1: Open Chat Interface
- [ ] **Action**: Click chat icon/button in dashboard header
- [ ] **Expected**: Chat interface opens (sidebar on desktop, modal on mobile)
- [ ] **Expected**: See empty chat with "Start a conversation" message
- [ ] **Expected**: See message input field at bottom
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 1.2: Send First Message
- [ ] **Action**: Type "What tasks do I have?" and press Enter (or click Send)
- [ ] **Expected**: Message appears immediately in chat (optimistic update)
- [ ] **Expected**: Loading indicator appears (typing animation)
- [ ] **Expected**: AI response appears within 5 seconds
- [ ] **Expected**: Response lists your tasks or says "You have no tasks"
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 1.3: Verify Message Display
- [ ] **Expected**: User message has blue background (right-aligned)
- [ ] **Expected**: AI message has gray background (left-aligned)
- [ ] **Expected**: Both messages show timestamps
- [ ] **Expected**: Messages are in chronological order
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 1.4: Close Chat
- [ ] **Action**: Click X button in chat header
- [ ] **Expected**: Chat closes smoothly with animation
- [ ] **Action**: Reopen chat and press Escape key
- [ ] **Expected**: Chat closes
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

---

### Test 2: Multi-Turn Conversation (User Story 2)

**Objective**: Verify conversation context is maintained across messages

#### Test 2.1: Create Task via Chat
- [ ] **Action**: Send "Create a task to buy milk"
- [ ] **Expected**: AI confirms task creation
- [ ] **Expected**: Conversation ID appears in header (e.g., "Conversation #1")
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 2.2: Follow-Up with Pronoun Reference
- [ ] **Action**: Send "Mark it as complete"
- [ ] **Expected**: AI understands "it" refers to the milk task
- [ ] **Expected**: AI confirms marking the task complete
- [ ] **Expected**: Same conversation ID in header
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 2.3: Another Follow-Up
- [ ] **Action**: Send "What's the status of that task?"
- [ ] **Expected**: AI understands "that task" refers to milk task
- [ ] **Expected**: AI responds with task status
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 2.4: New Conversation
- [ ] **Action**: Click "New Conversation" button in header
- [ ] **Expected**: Messages clear from chat
- [ ] **Expected**: Conversation ID resets or shows "New Conversation"
- [ ] **Action**: Send "What tasks do I have?"
- [ ] **Expected**: AI responds (should NOT reference milk task from previous conversation)
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 2.5: Conversation Persistence
- [ ] **Action**: Close chat and reopen it
- [ ] **Expected**: Previous messages are still visible
- [ ] **Expected**: Conversation ID is preserved
- [ ] **Action**: Send a follow-up message
- [ ] **Expected**: AI maintains context from before closing
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

---

### Test 3: Automatic Task List Refresh (User Story 3)

**Objective**: Verify task list automatically updates when AI modifies tasks

#### Test 3.1: Task Creation Refresh
- [ ] **Setup**: Open task list page (http://localhost:3000/dashboard/tasks)
- [ ] **Setup**: Open chat interface (keep both visible if possible)
- [ ] **Action**: Send "Create a task to buy groceries"
- [ ] **Expected**: AI confirms task creation
- [ ] **Expected**: "Task list updated" notification appears in chat
- [ ] **Expected**: Task list automatically refreshes within 1 second
- [ ] **Expected**: New "Buy groceries" task appears in list
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 3.2: Task Update Refresh
- [ ] **Action**: Send "Update the groceries task description to 'Buy organic groceries'"
- [ ] **Expected**: AI confirms update
- [ ] **Expected**: "Task list updated" notification appears
- [ ] **Expected**: Task list refreshes automatically
- [ ] **Expected**: Task description updates in list
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 3.3: Task Completion Refresh
- [ ] **Action**: Send "Mark the groceries task as complete"
- [ ] **Expected**: AI confirms completion
- [ ] **Expected**: "Task list updated" notification appears
- [ ] **Expected**: Task list refreshes automatically
- [ ] **Expected**: Task shows as completed (checkbox checked or strikethrough)
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 3.4: Task Deletion Refresh
- [ ] **Action**: Send "Delete the groceries task"
- [ ] **Expected**: AI confirms deletion
- [ ] **Expected**: "Task list updated" notification appears
- [ ] **Expected**: Task list refreshes automatically
- [ ] **Expected**: Task is removed from list
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

---

### Test 4: Responsive Design (User Story 4)

**Objective**: Verify chat works on all screen sizes

#### Test 4.1: Desktop Layout (1920px)
- [ ] **Action**: Resize browser to 1920px width
- [ ] **Action**: Open chat interface
- [ ] **Expected**: Chat opens as fixed sidebar on right side
- [ ] **Expected**: Chat width is approximately 384px (24rem)
- [ ] **Expected**: Chat doesn't obstruct task list
- [ ] **Expected**: Smooth slide-in animation from right
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 4.2: Tablet Layout (768px)
- [ ] **Action**: Resize browser to 768px width
- [ ] **Action**: Open chat interface
- [ ] **Expected**: Chat opens as modal overlay
- [ ] **Expected**: Chat takes most of screen but has margins
- [ ] **Expected**: Background is dimmed/overlayed
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 4.3: Mobile Layout (320px)
- [ ] **Action**: Resize browser to 320px width (or use mobile device)
- [ ] **Action**: Open chat interface
- [ ] **Expected**: Chat opens as bottom sheet (full width)
- [ ] **Expected**: Chat slides up from bottom
- [ ] **Expected**: Chat is usable on small screen
- [ ] **Expected**: Input field is accessible
- [ ] **Expected**: Messages are readable
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 4.4: Responsive Transitions
- [ ] **Action**: Open chat on desktop, then resize to mobile
- [ ] **Expected**: Chat layout adapts smoothly
- [ ] **Expected**: No layout breaks or overlaps
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

---

### Test 5: Error Handling

**Objective**: Verify graceful error handling in various failure scenarios

#### Test 5.1: Empty Message Validation
- [ ] **Action**: Try to send empty message (just spaces)
- [ ] **Expected**: Send button is disabled OR validation prevents sending
- [ ] **Expected**: No API request is made
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 5.2: Network Error
- [ ] **Action**: Stop backend server (Ctrl+C in backend terminal)
- [ ] **Action**: Send message in chat
- [ ] **Expected**: Error message displays in chat
- [ ] **Expected**: Error message is user-friendly (not technical)
- [ ] **Expected**: Message shows error icon/indicator
- [ ] **Action**: Restart backend server
- [ ] **Action**: Send another message
- [ ] **Expected**: Chat recovers and works normally
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 5.3: Backend Error (500)
- [ ] **Action**: Send a message that might cause backend error
- [ ] **Expected**: Error message displays
- [ ] **Expected**: Error doesn't crash the chat interface
- [ ] **Expected**: User can continue using chat after error
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 5.4: Expired Token
- [ ] **Action**: Manually delete or corrupt JWT token in localStorage
- [ ] **Action**: Send message
- [ ] **Expected**: 401 error is handled
- [ ] **Expected**: User is redirected to sign-in OR token refresh is attempted
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

---

### Test 6: Accessibility

**Objective**: Verify keyboard navigation and screen reader support

#### Test 6.1: Keyboard Navigation
- [ ] **Action**: Use Tab key to navigate dashboard
- [ ] **Expected**: Can reach chat open button with Tab
- [ ] **Expected**: Focus indicator is visible
- [ ] **Action**: Press Enter on chat button
- [ ] **Expected**: Chat opens
- [ ] **Action**: Press Tab inside chat
- [ ] **Expected**: Can navigate to input field
- [ ] **Expected**: Can navigate to close button
- [ ] **Action**: Press Escape
- [ ] **Expected**: Chat closes
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 6.2: Screen Reader Support
- [ ] **Action**: Enable screen reader (NVDA, JAWS, or VoiceOver)
- [ ] **Expected**: Chat button has descriptive label
- [ ] **Expected**: Messages are announced when received
- [ ] **Expected**: Input field has label
- [ ] **Expected**: Error messages are announced
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 6.3: Focus Management
- [ ] **Action**: Open chat
- [ ] **Expected**: Focus moves to input field automatically
- [ ] **Action**: Close chat
- [ ] **Expected**: Focus returns to chat open button
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

---

### Test 7: Performance

**Objective**: Verify performance meets requirements

#### Test 7.1: Chat Load Time
- [ ] **Action**: Open browser DevTools → Performance tab
- [ ] **Action**: Start recording
- [ ] **Action**: Click chat open button
- [ ] **Action**: Stop recording when chat is fully loaded
- [ ] **Expected**: Chat loads in < 2 seconds
- [ ] **Actual Time**: _____ seconds
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 7.2: Message Send/Receive Time
- [ ] **Action**: Open DevTools → Network tab
- [ ] **Action**: Send message
- [ ] **Action**: Note time from send to response received
- [ ] **Expected**: Complete in < 5 seconds (normal network)
- [ ] **Actual Time**: _____ seconds
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 7.3: Task Refresh Time
- [ ] **Action**: Send task-creating message
- [ ] **Action**: Measure time from AI response to task list update
- [ ] **Expected**: Task list refreshes in < 1 second
- [ ] **Actual Time**: _____ seconds
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

#### Test 7.4: Long Conversation Performance
- [ ] **Action**: Send 100+ messages (can use a script or manual)
- [ ] **Expected**: Chat remains responsive
- [ ] **Expected**: No noticeable lag when scrolling
- [ ] **Expected**: No memory leaks (check DevTools → Memory)
- [ ] **Result**: PASS / FAIL
- [ ] **Notes**: _____________________

---

## Browser Compatibility Testing

Test on multiple browsers:

### Chrome
- [ ] All tests pass
- [ ] **Version**: _____
- [ ] **Issues**: _____________________

### Firefox
- [ ] All tests pass
- [ ] **Version**: _____
- [ ] **Issues**: _____________________

### Safari (if available)
- [ ] All tests pass
- [ ] **Version**: _____
- [ ] **Issues**: _____________________

### Edge
- [ ] All tests pass
- [ ] **Version**: _____
- [ ] **Issues**: _____________________

---

## Test Summary

**Total Tests**: 35
**Passed**: _____
**Failed**: _____
**Blocked**: _____

**Pass Rate**: _____%

**Critical Issues Found**: _____

**Non-Critical Issues Found**: _____

---

## Issues Log

### Issue 1
- **Severity**: Critical / High / Medium / Low
- **Test**: _____
- **Description**: _____
- **Steps to Reproduce**: _____
- **Expected**: _____
- **Actual**: _____
- **Screenshot**: _____

### Issue 2
- **Severity**: Critical / High / Medium / Low
- **Test**: _____
- **Description**: _____
- **Steps to Reproduce**: _____
- **Expected**: _____
- **Actual**: _____
- **Screenshot**: _____

---

## Recommendations

Based on testing results:

1. _____________________
2. _____________________
3. _____________________

---

## Sign-Off

**Tester**: _____________________
**Date**: _____________________
**Status**: APPROVED / NEEDS FIXES / BLOCKED
**Next Steps**: _____________________
