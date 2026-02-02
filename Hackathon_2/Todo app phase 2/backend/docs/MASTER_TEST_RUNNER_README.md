# Master Test Runner - Quick Start Guide

## What This Does

This script automatically:
1. ✅ Starts the backend on port 8001 (in a new terminal window)
2. ✅ Runs Spec 005 tests (AI Agent Service - 8 scenarios)
3. ✅ Runs Spec 006 tests (Chat Endpoint - 9 scenarios)
4. ℹ️ Provides instructions for Spec 007 manual testing (Frontend Chat)

## Prerequisites

- Python 3.10+ installed
- All backend dependencies installed (`pip install -r requirements.txt`)
- Port 8001 available (not in use)

## How to Run

### Option 1: Simple (Recommended)

```powershell
cd "Z:\phse 33\backend"
python run_all_specs_tests.py
```

### Option 2: If you prefer to start backend manually

**Terminal 1 - Start Backend:**
```powershell
cd "Z:\phse 33\backend"
uvicorn main:app --reload --port 8001
```

**Terminal 2 - Run Tests:**
```powershell
cd "Z:\phse 33\backend"
python run_all_tests.py
python run_all_chat_tests.py
```

## What to Expect

### When you run the script:

1. **Backend starts** - A new terminal window opens with the backend running
   - Keep this window open during testing
   - You'll see FastAPI startup logs

2. **Spec 005 tests run** (~15 minutes)
   - 8 test scenarios for AI Agent Service
   - Tests agent tool calling, conversation context, error handling
   - You'll see colored output with ✓ PASS or ✗ FAIL for each test

3. **Spec 006 tests run** (~5 minutes)
   - 9 test scenarios for Chat Endpoint
   - Tests authentication, persistence, agent integration
   - You'll see colored output with ✓ PASS or ✗ FAIL for each test

4. **Spec 007 instructions displayed**
   - Manual testing required (browser interaction)
   - Instructions provided in the output

### Expected Output:

```
======================================================================
                    Master Test Runner - Specs 005, 006, 007
======================================================================

This script will:
  1. Start the backend on port 8001
  2. Run automated tests for Spec 005 (AI Agent Service)
  3. Run automated tests for Spec 006 (Chat Endpoint)
  4. Provide instructions for manual testing of Spec 007 (Frontend)

Press Enter to continue...

======================================================================
                        Starting Backend Server
======================================================================

ℹ Starting backend on port 8001...
ℹ This will open a new terminal window
ℹ Keep the terminal window open while tests run
ℹ Waiting for backend to start...
✓ Backend is healthy on port 8001
ℹ Service: task-management-auth-api
✓ Backend started successfully!

======================================================================
              Running Spec 005: AI Agent Service Tests
======================================================================

ℹ Running 8 test scenarios for AI Agent Service...
ℹ This will test: add_task, list_tasks, pronoun resolution, etc.

[Test output...]

✓ Spec 005 tests completed successfully!

======================================================================
                Running Spec 006: Chat Endpoint Tests
======================================================================

ℹ Running 9 test scenarios for Chat Endpoint...
ℹ This will test: authentication, persistence, agent integration, etc.

[Test output...]

✓ Spec 006 tests completed successfully!

======================================================================
    Spec 007: Frontend Chat Interface - Manual Testing Required
======================================================================

[Instructions for manual testing...]

======================================================================
                      Test Execution Summary
======================================================================

Automated Tests:
  Spec 005 (AI Agent Service): ✓ PASSED
  Spec 006 (Chat Endpoint):    ✓ PASSED

Manual Tests:
  Spec 007 (Frontend Chat):    ⏳ PENDING (requires manual testing)

Next Steps:
  1. All automated tests passed!
  2. Update tasks.md files to mark completed tests as [x]
  3. Proceed with manual testing for Spec 007 (see instructions above)
  4. After all tests complete, run: /sp.git.commit_pr

✓ Automated testing complete!
```

## After Tests Complete

### 1. Update Task Documentation

**Spec 005** - Mark these as `[x]` in `specs/005-ai-agent-service/tasks.md`:
- Line 66: T020 (add_task tool)
- Line 67: T021 (list_tasks tool)
- Line 86: T026 (pronoun resolution)
- Line 87: T027 (list reference)
- Line 107: T035 (error explanation)
- Line 108: T036 (clarifying questions)
- Line 127: T041 ("today" interpretation)
- Line 128: T042 ("tomorrow" interpretation)

**Spec 006** - Mark these as `[x]` in `specs/006-chat-endpoint/tasks.md`:
- Lines 79-86: T031-T038 (all authentication and persistence tests)

### 2. Manual Testing for Spec 007

Follow the instructions displayed by the script to test the frontend chat interface in your browser.

### 3. Commit Changes

After all tests pass:
```
/sp.git.commit_pr
```

## Troubleshooting

### Backend won't start
- Check if port 8001 is in use: `netstat -ano | findstr ":8001"`
- Kill the process if needed: `taskkill /PID <PID> /F`
- Try a different port by editing the script

### Tests fail
- Check backend logs in the terminal window
- Verify `.env` file has correct configuration
- Ensure database is accessible
- Check `GEMINI_API_KEY` is valid

### Script errors
- Ensure Python 3.10+ is installed: `python --version`
- Install dependencies: `pip install -r requirements.txt`
- Check file paths are correct

## Time Estimate

- Backend startup: 1 minute
- Spec 005 tests: 15 minutes
- Spec 006 tests: 5 minutes
- **Total automated**: ~20 minutes

- Spec 007 manual testing: 1.5-2.5 hours

## Files Created by This Process

- Test results in console output
- Backend logs in the terminal window
- No files are modified (you'll update tasks.md manually)

## Support

If you encounter issues:
1. Check the troubleshooting section above
2. Review backend logs in the terminal window
3. Check individual test scripts for more details:
   - `run_all_tests.py` (Spec 005)
   - `run_all_chat_tests.py` (Spec 006)
