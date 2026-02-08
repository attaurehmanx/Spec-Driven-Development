# Quick Start: Manual Testing Instructions

**For**: AI Agent Service (Feature 005)
**Time Required**: 15-20 minutes
**Difficulty**: Easy

## Step 1: Verify Prerequisites (2 minutes)

Open a terminal and navigate to the backend directory:

```bash
cd Z:\phse 33\backend
```

Check that your environment is configured:

```bash
# Check Python version (should be 3.10+)
python --version

# Verify .env file exists
dir .env

# Test database connection
python -c "from database.session import get_async_session; print('Database OK')"

# Test MCP tools
python -c "from mcp_server.tools.task_tools import add_task; print('MCP Tools OK')"
```

If any of these fail, refer to `TESTING_GUIDE.md` for troubleshooting.

## Step 2: Run Comprehensive Test Suite (10 minutes)

Execute all tests at once:

```bash
python run_all_tests.py
```

**What You'll See:**

1. Test execution progress for each test case
2. Agent responses and validation results
3. Performance metrics (iterations, execution time)
4. Final summary report with pass/fail status

**Expected Results:**

- All 8 tests should pass
- Success rate: 100%
- Average execution time: 1-2 seconds per test
- No errors or exceptions

## Step 3: Review Test Results (3 minutes)

Look for these key indicators:

### ✓ Success Indicators

- Status: "completed" for all tests
- Iterations: 1-5 per test (should not hit max of 15)
- Validation: "✓ PASS" for all test cases
- Agent responses are natural and user-friendly

### ✗ Failure Indicators

- Status: "error" or "max_iterations_reached"
- Validation: "✗ FAIL" or "✗ ERROR"
- Agent responses contain technical jargon or error codes
- Agent doesn't call appropriate tools

## Step 4: Run Individual Test Suites (Optional, 5 minutes)

If you want to test specific user stories:

```bash
# User Story 1: Basic conversation
python test_agent_manual.py
python test_agent_list_tasks.py

# User Story 2: Multi-turn context
python test_agent_multiturn.py

# User Story 3: Error handling
python test_agent_errors.py

# User Story 4: Time awareness
python test_agent_time_aware.py
```

## Step 5: Document Results (2 minutes)

Record your test results:

1. Open `TESTING_GUIDE.md`
2. Fill in the "Test Execution Summary" table
3. Note any issues in the "Issues Found" section
4. Save the file

## Step 6: Update Documentation (3 minutes)

If all tests pass:

1. Open `Z:\phse 33\specs\005-ai-agent-service\tasks.md`
2. Mark the following tasks as complete:
   - [x] T020 [US1] Verify agent correctly calls add_task tool
   - [x] T021 [US1] Verify agent correctly calls list_tasks tool
   - [x] T026 [US2] Verify agent references previous messages
   - [x] T027 [US2] Verify agent identifies "the first one"
   - [x] T035 [US3] Verify agent explains errors naturally
   - [x] T036 [US3] Verify agent asks clarifying questions
   - [x] T041 [US4] Verify agent interprets "today"
   - [x] T042 [US4] Verify agent interprets "tomorrow"
   - [x] T049 Validate all manual test scripts work end-to-end

3. Open `Z:\phse 33\specs\005-ai-agent-service\quickstart.md`
4. Replace example outputs with actual outputs from your test runs
5. Mark T050 as complete

## Quick Validation Checklist

Use this checklist to verify each test:

### T020: Add Task Tool Call
- [ ] Agent called `add_task` tool
- [ ] Response confirms task creation
- [ ] Response mentions "groceries"

### T021: List Tasks Tool Call
- [ ] Agent called `list_tasks` tool
- [ ] Response lists tasks or says "no tasks"
- [ ] Response is formatted naturally

### T026: Pronoun Resolution
- [ ] Agent understood "it" = "Call mom" task
- [ ] Agent called `complete_task` tool
- [ ] Response confirms completion

### T027: List Reference
- [ ] Agent understood "first one" = "Buy groceries"
- [ ] Agent called `delete_task` tool
- [ ] Response confirms deletion

### T035: Task Not Found Error
- [ ] Agent received error from tool
- [ ] Response explains error naturally (no technical terms)
- [ ] Response is helpful and polite

### T036: Ambiguous Request
- [ ] Agent recognized ambiguity
- [ ] Agent asked clarifying question
- [ ] Agent didn't guess or assume

### T041: Today Interpretation
- [ ] Agent checked current time
- [ ] Agent filtered by today's date
- [ ] Response references "today"

### T042: Tomorrow Interpretation
- [ ] Agent calculated tomorrow's date
- [ ] Agent created task for tomorrow
- [ ] Response confirms future date

## Troubleshooting Quick Reference

| Error | Quick Fix |
|-------|-----------|
| Invalid API key | Check `GEMINI_API_KEY` in `.env` |
| User not found | Create test user in database |
| Max iterations | Check system prompt clarity |
| Tool not found | Verify MCP tools are imported |
| Database error | Check `DATABASE_URL` in `.env` |

## What to Do Next

### If All Tests Pass ✓

1. Mark T049 and T050 as complete in `tasks.md`
2. Commit your changes
3. Proceed to Chat API integration
4. Celebrate! The agent service is working correctly.

### If Tests Fail ✗

1. Review failure details in test output
2. Check `TESTING_GUIDE.md` for detailed troubleshooting
3. Fix issues in agent service implementation
4. Re-run tests
5. Document issues for future reference

## Need Help?

- **Detailed Guide**: See `TESTING_GUIDE.md`
- **Test Scripts**: Located in `Z:\phse 33\backend\test_agent_*.py`
- **Agent Service**: `Z:\phse 33\backend\services\agent_service.py`
- **Configuration**: `Z:\phse 33\backend\config\agent_config.py`
- **Spec**: `Z:\phse 33\specs\005-ai-agent-service\spec.md`

---

**Ready to Test**: Execute `python run_all_tests.py` to begin
