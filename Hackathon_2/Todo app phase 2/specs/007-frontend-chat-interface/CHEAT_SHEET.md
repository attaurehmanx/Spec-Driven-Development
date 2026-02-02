# Testing Cheat Sheet - Frontend Chat Interface

**Keep this open while testing**

---

## Services Running?

```bash
# Backend: http://localhost:8000/docs
# Frontend: http://localhost:3000
```

---

## 12 Tests to Complete

### 1. Basic Chat ✅
- [ ] Click chat icon in header
- [ ] Send: "What tasks do I have?"
- [ ] AI responds within 5 seconds
- [ ] Messages display correctly

### 2. Multi-Turn ✅
- [ ] Send: "Create a task to buy milk"
- [ ] Send: "Mark it as complete"
- [ ] AI understands "it" = milk task

### 3. Task Creation + Auto-Refresh ✅
- [ ] Send: "Create a task to test chat"
- [ ] "Task list updated" notification appears
- [ ] Task list refreshes automatically
- [ ] New task appears in list

### 4. Task Update + Auto-Refresh ✅
- [ ] Send: "Update [task] description to [new text]"
- [ ] Task list refreshes
- [ ] Changes appear

### 5. Task Deletion + Auto-Refresh ✅
- [ ] Send: "Delete [task]"
- [ ] Task list refreshes
- [ ] Task removed

### 6. Empty Message Validation ✅
- [ ] Try to send empty message
- [ ] Button disabled or validation prevents

### 7. Network Error ✅
- [ ] Stop backend (Ctrl+C)
- [ ] Send message
- [ ] Error message displays
- [ ] Restart backend, verify recovery

### 8. Expired Token ✅
- [ ] Delete token from localStorage
- [ ] Send message
- [ ] Refresh or redirect to sign-in

### 9. Responsive - Desktop (1920px) ✅
- [ ] Resize to 1920px
- [ ] Chat opens as sidebar on right
- [ ] 384px width, doesn't obstruct list

### 10. Responsive - Tablet (768px) ✅
- [ ] Resize to 768px
- [ ] Chat opens as modal
- [ ] Background dimmed

### 11. Responsive - Mobile (320px) ✅
- [ ] Resize to 320px
- [ ] Chat opens as bottom sheet
- [ ] Full width, usable

### 12. Accessibility ✅
- [ ] Tab to chat button
- [ ] Enter opens chat
- [ ] Escape closes chat
- [ ] Focus moves to input

### 13. Performance ✅
- [ ] Chat loads < 2 seconds
- [ ] Messages < 5 seconds
- [ ] Task refresh < 1 second

### 14. Long Conversation ✅
- [ ] Send 100+ messages
- [ ] No lag or degradation

---

## Quick Commands

### Start Backend
```bash
cd "Z:\phse 33\backend"
venv\Scripts\activate
uvicorn main:app --reload --port 8000
```

### Start Frontend
```bash
cd "Z:\phse 33\frontend-app"
npm run dev
```

### Check Backend
```bash
curl http://localhost:8000/health
```

---

## Browser DevTools (F12)

### Console Tab
- Check for errors
- Look for "tasks-updated" event

### Network Tab
- Monitor POST to `/api/{user_id}/chat`
- Check response times
- Verify Authorization header

### Application Tab
- Check localStorage for `access_token`
- Verify conversation_id stored

---

## Common Issues

### Chat button not visible
→ Check dashboard layout includes chat toggle

### Messages don't send
→ Check console for errors
→ Check Network tab for API request
→ Verify JWT token in localStorage

### Task list doesn't refresh
→ Check console for "tasks-updated" event
→ Verify task list uses useTaskRefresh hook

### 401 Unauthorized
→ Sign out and sign in again

---

## Performance Targets

| Metric | Target |
|--------|--------|
| Chat Load | < 2s |
| Message | < 5s |
| Refresh | < 1s |

---

## Files to Update After Testing

### tasks.md (lines 281-292)
Change `- [ ]` to `- [x]`

### IMPLEMENTATION_SUMMARY.md (lines 189-235)
Change `- [ ]` to `- [x]` + add results

---

## Test Progress

**Completed**: _____ / 12
**Status**: PASS / FAIL / IN PROGRESS

---

## Quick Smoke Test (5 min)

1. Sign in
2. Go to dashboard
3. Click chat icon
4. Send: "What tasks do I have?"
5. Send: "Create a task to test"
6. Verify task list refreshes
7. Close and reopen chat
8. Messages still visible

**All pass?** → Continue detailed testing
**Any fail?** → Check troubleshooting

---

## URLs

- Backend: http://localhost:8000/docs
- Frontend: http://localhost:3000
- Sign In: http://localhost:3000/sign-in
- Dashboard: http://localhost:3000/dashboard

---

## Documentation

All in: `Z:\phse 33\specs\007-frontend-chat-interface\`

- **QUICK_START.md** - Start here
- **TESTING_GUIDE.md** - Detailed tests
- **TEST_RESULTS_TEMPLATE.md** - Record results
- **SERVICE_STARTUP.md** - Troubleshooting

---

**Ready?** Start services → Execute tests → Record results
