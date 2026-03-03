# AI Agent Service - Manual Testing Guide

**Feature**: 005-ai-agent-service
**Date**: 2026-01-30
**Status**: Ready for Manual Testing

## Overview

This guide provides step-by-step instructions for executing manual tests of the AI Agent Service. All implementation tasks (T001-T048) are complete. The remaining tasks (T020, T021, T026, T027, T035, T036, T041, T042, T049, T050) are **manual verification tests** that require human execution and observation.

## Prerequisites

### 1. Environment Setup

Ensure the following are configured in `Z:\phse 33\backend\.env`:

```bash
# Database
DATABASE_URL=postgresql://neondb_owner:npg_UnmYjF5c7Sdo@ep-tiny-sea-a4v0nlab-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require

# Gemini API
GEMINI_API_KEY=AIzaSyAAIkAnf-Z-6GC8MBMYRo8dszRYrYBQB-Q
GEMINI_MODEL=gemini-2.5-flash
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/

# Agent Configuration
AGENT_MAX_ITERATIONS=15
AGENT_TEMPERATURE=1.0
AGENT_MAX_TOKENS=1000
AGENT_TIMEOUT=30
```

### 2. Database Access

Verify database connection:

```bash
cd Z:\phse 33\backend
python -c "from database.session import get_async_session; print('✓ Database accessible')"
```

### 3. Dependencies

Ensure all dependencies are installed:

```bash
cd Z:\phse 33\backend
pip install -r requirements.txt
```

### 4. Test User

The tests use `user_id = "test-user-123"`. This user should exist in your database, or the tests will create tasks for a non-existent user (which may cause errors depending on your database constraints).

## Test Execution Methods

### Method 1: Comprehensive Test Runner (Recommended)

Run all tests at once with detailed reporting:

```bash
cd Z:\phse 33\backend
python run_all_tests.py
```

**Expected Output:**
- Test execution summary
- Pass/fail status for each test
- Detailed validation results
- Performance metrics (iterations, execution time)
- Overall success rate

**What to Look For:**
- All 8 tests should pass (T020, T021, T026, T027, T035, T036, T041, T042)
- Success rate should be 100%
- Execution time should be <3 seconds per test (excluding LLM latency)
- Agent should complete within 15 iterations for all tests

### Method 2: Individual Test Scripts

Run specific test suites:

#### User Story 1: Basic Agent Conversation
```bash
cd Z:\phse 33\backend
python test_agent_manual.py
python test_agent_list_tasks.py
```

**Tests Covered:**
- T020: Verify add_task tool call
- T021: Verify list_tasks tool call

#### User Story 2: Multi-Turn Conversation
```bash
cd Z:\phse 33\backend
python test_agent_multiturn.py
```

**Tests Covered:**
- T026: Pronoun resolution ("Mark it as done")
- T027: List reference ("Delete the first one")

#### User Story 3: Error Handling
```bash
cd Z:\phse 33\backend
python test_agent_errors.py
```

**Tests Covered:**
- T035: Task not found error explanation
- T036: Ambiguous request clarification

#### User Story 4: Time-Aware Management
```bash
cd Z:\phse 33\backend
python test_agent_time_aware.py
```

**Tests Covered:**
- T041: "Today" interpretation
- T042: "Tomorrow" interpretation

## Manual Verification Checklist

For each test, verify the following:

### T020: Add Task Tool Call
- [ ] Agent calls `add_task` tool (not just responding without tool use)
- [ ] Tool call includes correct parameters (title, user_id)
- [ ] Agent confirms task creation in natural language
- [ ] Response mentions the task title ("groceries")

### T021: List Tasks Tool Call
- [ ] Agent calls `list_tasks` tool
- [ ] Tool call includes user_id parameter
- [ ] Agent formats task list in readable format
- [ ] Response includes task details (title, status)

### T026: Pronoun Resolution
- [ ] Agent understands "it" refers to previously created task
- [ ] Agent calls `complete_task` with correct task_id
- [ ] Agent doesn't ask "which task?" (context is clear)
- [ ] Response confirms completion of the correct task

### T027: List Reference Resolution
- [ ] Agent understands "the first one" from previous list
- [ ] Agent identifies correct task_id from conversation history
- [ ] Agent calls `delete_task` with correct task_id
- [ ] Response confirms deletion of "Buy groceries"

### T035: Task Not Found Error
- [ ] Agent receives error from tool execution
- [ ] Agent explains error in user-friendly language (no technical jargon)
- [ ] Response doesn't include "TaskNotFoundError" or stack traces
- [ ] Agent suggests helpful next steps

### T036: Ambiguous Request Clarification
- [ ] Agent recognizes request is ambiguous
- [ ] Agent asks clarifying question before taking action
- [ ] Question is specific (e.g., "Which task would you like to update?")
- [ ] Agent doesn't attempt to guess or make assumptions

### T041: Today Interpretation
- [ ] Agent checks current time from system prompt
- [ ] Agent correctly interprets "today" as current date
- [ ] Agent filters tasks by current date
- [ ] Response references today or current date

### T042: Tomorrow Interpretation
- [ ] Agent checks current time from system prompt
- [ ] Agent correctly calculates tomorrow's date
- [ ] Agent creates task with tomorrow's due date
- [ ] Response confirms task creation for tomorrow

## Performance Validation

For each test, verify:

- **Iterations**: Should complete in ≤5 iterations for simple queries
- **Execution Time**: Should be <3 seconds (excluding LLM API latency)
- **Status**: Should be "completed" (not "max_iterations_reached" or "error")
- **Tool Calls**: Should use appropriate tools (not hallucinating responses)

## Common Issues and Troubleshooting

### Issue: "Invalid API key"

**Symptoms:**
```
LLMAPIError: Invalid API key
```

**Solution:**
1. Verify `GEMINI_API_KEY` in `.env` file
2. Check for extra spaces or quotes
3. Test API key at https://aistudio.google.com/

### Issue: "User not found"

**Symptoms:**
```
Tool execution failed: User test-user-123 not found
```

**Solution:**
1. Create test user in database:
```python
from models.task_models import User
from database.session import get_async_session

async def create_test_user():
    async for session in get_async_session():
        user = User(
            id="test-user-123",
            email="test@example.com",
            username="testuser"
        )
        session.add(user)
        await session.commit()
```

### Issue: "Max iterations exceeded"

**Symptoms:**
```
Status: max_iterations_reached
Warning: Agent reached maximum iterations
```

**Solution:**
1. Check system prompt clarity
2. Verify tool schemas are correct
3. Review agent loop logic
4. Increase `AGENT_MAX_ITERATIONS` if needed

### Issue: "Tool not found"

**Symptoms:**
```
Tool execution failed: Unknown tool: add_task
```

**Solution:**
1. Verify MCP tools are accessible:
```bash
python -c "from mcp_server.tools.task_tools import add_task; print('✓ Tools loaded')"
```
2. Check TOOL_REGISTRY in `services/agent_service.py`

## Test Results Documentation

After running tests, document results in the following format:

### Test Execution Summary

**Date**: [YYYY-MM-DD]
**Tester**: [Your Name]
**Environment**: [Development/Staging/Production]

| Test ID | Test Name | Status | Iterations | Time (ms) | Notes |
|---------|-----------|--------|------------|-----------|-------|
| T020 | Add task tool call | PASS | 2 | 1234 | Agent correctly called add_task |
| T021 | List tasks tool call | PASS | 2 | 1156 | Formatted response well |
| T026 | Pronoun resolution | PASS | 3 | 1567 | Resolved "it" correctly |
| T027 | List reference | PASS | 3 | 1489 | Identified "first one" |
| T035 | Task not found error | PASS | 2 | 1234 | User-friendly error message |
| T036 | Ambiguous clarification | PASS | 1 | 987 | Asked clarifying question |
| T041 | Today interpretation | PASS | 2 | 1345 | Correctly filtered by date |
| T042 | Tomorrow interpretation | PASS | 2 | 1278 | Created task for tomorrow |

**Overall Success Rate**: 8/8 (100%)

### Issues Found

Document any issues discovered during testing:

1. **Issue**: [Description]
   - **Severity**: [Critical/High/Medium/Low]
   - **Test**: [Test ID]
   - **Expected**: [Expected behavior]
   - **Actual**: [Actual behavior]
   - **Reproduction**: [Steps to reproduce]

## Next Steps After Testing

### If All Tests Pass (T049 Complete)

1. Mark T049 as complete in `tasks.md`
2. Update `quickstart.md` with actual test output examples (T050)
3. Document test results in this guide
4. Proceed to integration with Chat API

### If Tests Fail

1. Document failure details
2. Review agent service implementation
3. Check system prompt effectiveness
4. Verify tool bindings
5. Re-test after fixes

## Final Validation (T049)

Before marking testing complete, verify:

- [ ] All 8 manual tests execute successfully
- [ ] Agent correctly calls all 5 MCP tools (add_task, list_tasks, complete_task, update_task, delete_task)
- [ ] Agent maintains conversation context across multiple turns
- [ ] Agent handles all error scenarios with user-friendly messages
- [ ] Agent correctly interprets time-relative queries (today, tomorrow)
- [ ] Agent enforces user isolation (never accesses other users' tasks)
- [ ] Agent completes within 15 iterations for typical requests
- [ ] Response time <3 seconds for 95% of requests (excluding LLM latency)
- [ ] All functions have docstrings and type hints
- [ ] Configuration is externalized in .env file

## Update Quickstart.md (T050)

After completing manual tests, update `specs/005-ai-agent-service/quickstart.md` with:

1. **Actual test output examples** from your test runs
2. **Performance metrics** (iterations, execution time)
3. **Common issues encountered** during testing
4. **Tips for optimal agent performance**

Replace placeholder examples with real output from `run_all_tests.py`.

---

**Testing Guide Complete**: Ready for manual test execution
