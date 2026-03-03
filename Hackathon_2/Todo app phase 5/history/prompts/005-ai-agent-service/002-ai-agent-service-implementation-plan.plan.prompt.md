---
id: 002
title: AI Agent Service Implementation Plan
stage: plan
date: 2026-01-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: 005-ai-agent-service
branch: 005-ai-agent-service
user: HP
command: /sp.plan
labels: ["planning", "ai-agent", "gemini-api", "mcp-tools", "agent-loop", "architecture"]
links:
  spec: specs/005-ai-agent-service/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-ai-agent-service/plan.md
tests:
  - none (planning phase)
---

## Prompt

```text
1.  Setup `agent_service.py`.
2.  Configure Gemini API Client.
3.  Implement the recursive Tool Execution Loop (The Agent "Runner").
4.  Test with a mock prompt to ensure it calls `add_task` correctly.
5.  First explore the backend then make a plan.
```

## Response snapshot

Created comprehensive implementation plan for AI Agent Service Configuration (005-ai-agent-service):

**Plan Structure:**
- **Summary**: Stateless agent runner using OpenAI SDK configured for Gemini API, binding 5 MCP tools
- **Technical Context**: Python 3.10+, OpenAI SDK, FastMCP, pytest-asyncio, <3s response time target
- **Constitution Check**: All 9 principles validated (PASSED)
- **Project Structure**: Backend service layer with agent_service.py, agent_config.py, test files

**Phase 0 - Research (5 Tasks)**:
1. Gemini API OpenAI compatibility configuration
2. MCP tool to OpenAI function format conversion
3. Agent loop implementation pattern (iterative with max 15 iterations)
4. Error handling strategies (LLM failures, tool errors)
5. Message format conversion (DB to OpenAI chat format)

**Phase 1 - Design**:
- Data models: AgentConfiguration, SystemPrompt, AgentLoopConfig, ToolBinding
- Contracts: run_agent(), bind_mcp_tools(), execute_tool_call()
- Quickstart guide with setup, testing, and troubleshooting

**Key Architectural Decisions**:
- Stateless agent service (Principle VII compliance)
- User_id validation in system prompt and all tool calls (Principle IX)
- Max 15 iterations with graceful timeout
- Synchronous operation (no streaming for MVP)
- Error handling returns user-friendly messages

**Backend Exploration Findings**:
- Existing MCP tools: add_task, list_tasks, complete_task, update_task, delete_task
- MCP server instance at backend/mcp_server/server.py
- Service layer pattern with user_service.py (agent_service.py will follow same pattern)
- FastAPI main.py with CORS, authentication, task/user endpoints

**Risk Analysis**:
- Medium risk: Gemini API OpenAI compatibility (mitigated by Phase 0 research)
- Low risk: Tool binding complexity (Pydantic schemas already exist)
- Medium risk: LLM rate limits (graceful error handling)

## Outcome

- ✅ Impact: Complete implementation plan ready for task breakdown
- 🧪 Tests: Planning phase - no tests yet (test strategy defined in plan)
- 📁 Files: Created comprehensive plan.md with Phase 0 research tasks and Phase 1 design artifacts
- 🔁 Next prompts: Execute Phase 0 research tasks, then `/sp.tasks` to generate task breakdown
- 🧠 Reflection: Plan successfully bridges specification to implementation. Backend exploration revealed clean service layer pattern to follow. Research phase will validate Gemini API compatibility before implementation begins.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning workflow executed successfully
- Graders run and results (PASS/FAIL): Constitution check PASS (9/9 principles), Project structure PASS (web app backend pattern), Risk analysis PASS (5 risks identified with mitigations)
- Prompt variant (if applicable): Standard /sp.plan workflow with backend exploration
- Next experiment (smallest change to try): Execute Phase 0 research tasks to validate Gemini API OpenAI compatibility and tool binding patterns before proceeding to task breakdown
