# Manual Testing - Next Steps

**Feature**: 007-frontend-chat-interface
**Date**: 2026-01-30
**Status**: Testing Documentation Complete - Ready for Manual Execution

---

## What Has Been Prepared

I've created comprehensive testing documentation to guide you through the manual testing process:

### 1. Testing Guide (`TESTING_GUIDE.md`)
- **35 detailed test cases** covering all user stories
- Step-by-step instructions for each test
- Expected results and pass/fail criteria
- Organized by feature area (Basic Chat, Multi-Turn, Auto-Refresh, Responsive, Error Handling, Accessibility, Performance)

### 2. Test Results Template (`TEST_RESULTS_TEMPLATE.md`)
- Structured template for recording test results
- Space for screenshots and notes
- Issue tracking section
- Performance metrics tracking
- Browser compatibility checklist

### 3. Service Startup Guide (`SERVICE_STARTUP.md`)
- Commands to start backend and frontend services
- Troubleshooting common startup issues
- Environment variable reference
- Verification checklist

---

## What You Need to Do

### Step 1: Start the Services

**Open Terminal 1 - Backend**:
```bash
cd "Z:\phse 33\backend"
venv\Scripts\activate
uvicorn main:app --reload --port 8000
```

**Open Terminal 2 - Frontend**:
```bash
cd "Z:\phse 33\frontend-app"
npm run dev
```

**Verify Both Are Running**:
- Backend: http://localhost:8000/docs (should see Swagger docs)
- Frontend: http://localhost:3000 (should see homepage)

### Step 2: Sign In

1. Navigate to: http://localhost:3000/sign-in
2. Sign in with your test account
3. Should redirect to: http://localhost:3000/dashboard
4. Verify you see the task list

### Step 3: Execute Manual Tests

**Open the Testing Guide**:
```bash
# Open in your preferred editor
code "Z:\phse 33\specs\007-frontend-chat-interface\TESTING_GUIDE.md"
```

**Open the Test Results Template**:
```bash
# Make a copy for your test session
copy "Z:\phse 33\specs\007-frontend-chat-interface\TEST_RESULTS_TEMPLATE.md" "Z:\phse 33\specs\007-frontend-chat-interface\TEST_RESULTS_2026-01-30.md"
```

**Execute Tests**:
1. Follow each test case in TESTING_GUIDE.md
2. Record results in TEST_RESULTS_2026-01-30.md
3. Take screenshots of any issues
4. Note performance metrics

### Step 4: Update Documentation

After completing tests, update these files:

**1. Update tasks.md checklist**:
```bash
# Open tasks.md
code "Z:\phse 33\specs\007-frontend-chat-interface\tasks.md"

# Mark completed tests as [x]:
- [x] Basic chat: Send message, receive response
- [x] Multi-turn: Send follow-up with pronoun reference, AI understands context
# ... etc
```

**2. Update IMPLEMENTATION_SUMMARY.md**:
```bash
# Open IMPLEMENTATION_SUMMARY.md
code "Z:\phse 33\specs\007-frontend-chat-interface\IMPLEMENTATION_SUMMARY.md"

# Update the "Manual Testing Checklist" section (lines 189-235)
# Change [ ] to [x] for completed tests
```

---

## Testing Priority

If you have limited time, test in this order:

### Priority 1: Critical Path (Must Test)
1. **Basic Chat** (Test 1.1-1.4)
   - Open chat, send message, receive response
2. **Multi-Turn Conversation** (Test 2.1-2.2)
   - Create task, follow up with pronoun reference
3. **Auto-Refresh** (Test 3.1)
   - Create task via chat, verify list refreshes

### Priority 2: Core Features (Should Test)
4. **Responsive Design** (Test 4.1-4.3)
   - Desktop, tablet, mobile layouts
5. **Error Handling** (Test 5.1-5.2)
   - Empty message validation, network error

### Priority 3: Quality Assurance (Nice to Test)
6. **Accessibility** (Test 6.1)
   - Keyboard navigation
7. **Performance** (Test 7.1-7.3)
   - Load time, message time, refresh time

---

## Quick Test Checklist

Use this for a rapid smoke test (5-10 minutes):

- [ ] Backend running at http://localhost:8000
- [ ] Frontend running at http://localhost:3000
- [ ] Can sign in successfully
- [ ] Dashboard loads with task list
- [ ] Chat icon visible in header
- [ ] Click chat icon → chat opens
- [ ] Send "What tasks do I have?" → AI responds
- [ ] Send "Create a task to test chat" → AI creates task
- [ ] Task list automatically refreshes
- [ ] New task appears in list
- [ ] Close chat with X button
- [ ] Reopen chat → messages still visible
- [ ] Resize browser to mobile size → chat adapts
- [ ] No console errors in browser DevTools

**If all above pass**: Feature is working! Proceed with detailed testing.
**If any fail**: Check SERVICE_STARTUP.md troubleshooting section.

---

## Common Issues and Solutions

### Issue: Chat button not visible
**Solution**: Check that dashboard layout includes chat toggle button
**File**: `Z:\phse 33\frontend-app\app\dashboard\layout.tsx`

### Issue: Chat opens but messages don't send
**Solution**:
1. Check browser console for errors
2. Check Network tab for API request
3. Verify JWT token in localStorage
4. Check backend logs for errors

### Issue: Task list doesn't refresh
**Solution**:
1. Open browser console
2. Send task-creating message
3. Look for "tasks-updated" event dispatch
4. Verify task list component is listening

### Issue: Backend returns 401 Unauthorized
**Solution**:
1. Sign out and sign in again
2. Check JWT token in localStorage
3. Verify BETTER_AUTH_SECRET matches in backend and frontend

---

## Browser DevTools Setup

Keep these tabs open while testing:

1. **Console Tab**
   - Monitor for errors and warnings
   - Look for "tasks-updated" event logs

2. **Network Tab**
   - Monitor API requests to `/api/{user_id}/chat`
   - Check request/response times
   - Verify Authorization header includes JWT token

3. **Application Tab**
   - Check localStorage for `access_token`
   - Verify conversation_id is stored

4. **Performance Tab** (for performance tests)
   - Record chat load time
   - Measure message send/receive time

---

## Test Execution Tips

### Taking Screenshots
- Use Windows Snipping Tool (Win + Shift + S)
- Or browser DevTools screenshot (Ctrl + Shift + P → "Screenshot")
- Save to: `Z:\phse 33\specs\007-frontend-chat-interface\screenshots\`

### Recording Performance
- Use browser DevTools Performance tab
- Start recording before action
- Stop after action completes
- Note the time in milliseconds

### Testing Responsive Design
- Use browser DevTools Device Toolbar (Ctrl + Shift + M)
- Or manually resize browser window
- Test these widths: 320px, 768px, 1920px

### Testing Accessibility
- Use keyboard only (no mouse)
- Tab to navigate, Enter to activate, Escape to close
- Use screen reader if available (NVDA, JAWS, VoiceOver)

---

## After Testing

### If All Tests Pass
1. Mark all checklist items as [x] in tasks.md
2. Update IMPLEMENTATION_SUMMARY.md with "All tests passed"
3. Fill in performance metrics
4. Sign off on TEST_RESULTS document
5. Feature is ready for production!

### If Tests Fail
1. Document each failure in TEST_RESULTS
2. Include screenshots and error messages
3. Note severity (Critical, High, Medium, Low)
4. Create list of issues to fix
5. Report findings to development team

---

## Files Reference

All testing documentation is in: `Z:\phse 33\specs\007-frontend-chat-interface\`

| File | Purpose |
|------|---------|
| `TESTING_GUIDE.md` | Detailed test cases and instructions |
| `TEST_RESULTS_TEMPLATE.md` | Template for recording results |
| `SERVICE_STARTUP.md` | How to start backend and frontend |
| `NEXT_STEPS.md` | This file - what to do next |
| `tasks.md` | Task list with testing checklist |
| `IMPLEMENTATION_SUMMARY.md` | Implementation status and results |

---

## Time Estimate

- **Quick Smoke Test**: 5-10 minutes
- **Priority 1 Tests**: 15-20 minutes
- **Priority 1 + 2 Tests**: 30-45 minutes
- **Full Test Suite**: 1-2 hours
- **Multi-Browser Testing**: +30 minutes per browser

---

## Support

If you encounter issues during testing:

1. **Check SERVICE_STARTUP.md** for troubleshooting
2. **Check browser console** for error messages
3. **Check backend logs** in terminal
4. **Check Network tab** for failed API requests
5. **Verify environment variables** in .env files

---

## Ready to Start?

1. ✅ Testing documentation prepared
2. ⏳ Start backend service
3. ⏳ Start frontend service
4. ⏳ Sign in to application
5. ⏳ Execute tests from TESTING_GUIDE.md
6. ⏳ Record results in TEST_RESULTS template
7. ⏳ Update tasks.md checklist
8. ⏳ Update IMPLEMENTATION_SUMMARY.md

**Let's begin!** Open SERVICE_STARTUP.md and start the services.
