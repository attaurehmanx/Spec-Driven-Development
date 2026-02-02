---
id: 003
title: Update constitution for Phase 3 AI chatbot
stage: constitution
date: 2026-01-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: unknown
user: unknown
command: /sp.constitution
labels: ["constitution", "phase-3", "ai-chatbot", "principles", "mcp", "agents-sdk"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
# Project Constitution: Todo AI Chatbot (Phase 3)

## 1. Project Objective
Extend the existing Phase 2 Full-Stack Todo application into an AI-powered conversational interface. The system will allow users to manage tasks (add, list, update, delete) via natural language using an Agentic architecture.

## 2. Architecture Overview
The system follows a stateless, tool-use architecture:
1.  **Frontend:** OpenAI ChatKit (extending existing next.js frontend) for the chat UI.
2.  **Backend:** Python FastAPI (extending existing backend).
3.  **AI Layer:** OpenAI Agents SDK (configured with Gemini API as the LLM provider).
4.  **Tool Layer:** Model Context Protocol (MCP) Server (Official Python SDK).
5.  **Data Layer:** Neon Serverless PostgreSQL (SQLModel ORM).

## 3. Technology Stack & Constraints
* **Language:** Python 3.10+ (Backend), TypeScript/Next.js (Frontend).
* **ORM:** SQLModel (must align with existing `Task` model).
* **Auth:** Better Auth (maintain existing JWT flow; Chat endpoints are protected).
* **MCP:** Official `mcp` Python SDK.
* **LLM Provider:** Google Gemini (via API Key) adapted to the Agent framework.
* **Development Workflow:** Spec-Driven Development (Spec -> Plan ->Task-> Implementation).

## 4. Global Data Rules
* **Statelessness:** The Chat API endpoint (`POST /api/chat`) must be stateless.
* **Persistence:** Conversation history (User & Assistant messages) must be persisted to the DB *before* and *after* the agent processes the request.
* **User Isolation:** All operations (listing, adding, deleting) must strictly filter by `user_id`. An agent must never access data from another user.

## 5. Coding Standards
* **Type Safety:** Strict Pydantic models for all API inputs/outputs and MCP tools.
* **Async:** All Database and API I/O must be `async/await`.
* **Error Handling:** Agents must degrade gracefully (e.g., "I couldn't find that task") rather than crashing the server.
```

## Response snapshot

Updated the project constitution from version 1.1.0 to 2.0.0 (MAJOR version bump) to reflect the architectural evolution from Phase 2 (Multi-User Todo Full-Stack Web Application) to Phase 3 (Todo AI Chatbot).

**Key Changes:**
- Added three new core principles:
  * Principle VII: Stateless AI interactions - Chat API must be stateless
  * Principle VIII: Conversation persistence - All messages must be persisted to DB
  * Principle IX: User data isolation in AI context - AI agents must enforce user_id filtering
- Updated technology stack to include:
  * Frontend: OpenAI ChatKit
  * AI Layer: OpenAI Agents SDK with Gemini API
  * Tool Layer: Model Context Protocol (MCP) Server
- Added new AI/Agent Standards section covering:
  * Stateless chat endpoint requirements
  * Conversation history persistence rules
  * MCP tool type safety and async requirements
  * Agent error handling standards
- Updated Success Criteria to reflect AI chatbot capabilities
- Added AI Safety Review to Definition of Done checklist

**Version Bump Rationale:**
MAJOR (2.0.0) - This represents a backward-incompatible architectural shift from a traditional REST API application to an AI-powered conversational interface. The addition of the AI layer and MCP tools fundamentally changes the system architecture and introduces new principles that affect all future development.

**Template Alignment:**
- ✅ spec-template.md - Aligned (spec-first requirement matches principle I)
- ✅ plan-template.md - Aligned (constitution check gate present)
- ✅ tasks-template.md - Aligned (task organization matches principles)
- ⚠ commands/*.md - No command files found, no updates needed

## Outcome

- ✅ Impact: Constitution successfully updated to version 2.0.0 with three new principles for AI chatbot architecture
- 🧪 Tests: N/A (constitution document)
- 📁 Files: 1 file modified (.specify/memory/constitution.md)
- 🔁 Next prompts: Create feature specification for AI chatbot using /sp.specify
- 🧠 Reflection: The constitution now provides clear governance for the AI layer, ensuring stateless design, conversation persistence, and user data isolation are enforced from the start

## Evaluation notes (flywheel)

- Failure modes observed: None - constitution update completed successfully
- Graders run and results (PASS/FAIL): N/A (manual review required)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate that new principles are enforceable during implementation phase
