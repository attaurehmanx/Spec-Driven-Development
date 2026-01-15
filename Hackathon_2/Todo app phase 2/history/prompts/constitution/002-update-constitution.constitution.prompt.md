---
id: 002
title: Update Constitution
stage: constitution
date: 2026-01-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-frontend-app
user: user
command: /sp.constitution
labels: ["constitution", "governance", "principles"]
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
/sp.constitution

Project: Phase II – Multi-User Todo Full-Stack Web Application

Core principles:
- Spec-first development: No planning or implementation without an approved spec
- Single responsibility per spec: No overlapping domains
- Explicit contracts: All cross-boundary behavior must be defined, not assumed
- Security by default: All APIs are authenticated unless explicitly stated otherwise
- Determinism: Same spec and prompts must produce equivalent outputs
- Agentic discipline: All code generated via Claude Code; no manual coding

Key standards:
- All functionality must trace back to an approved spec
- REST APIs must be stateless and JWT-protected
- User data isolation must be enforced at the query level
- Frontend and backend communicate only via defined API contracts
- Environment-based configuration for all secrets
- No hidden coupling between frontend and backend implementations

Technology constraints:
- Frontend: Next.js 16+ with App Router
- Backend: Python FastAPI
- ORM: SQLModel
- Database: Neon Serverless PostgreSQL
- Authentication: Better Auth (JWT-based)
- Auth transport: Authorization: Bearer <token>

Security standards:
- All endpoints require a valid JWT after authentication is introduced
- Backend must verify JWT signature and expiry
- User identity must be derived exclusively from JWT claims
- URL user_id must match authenticated user identity
- Requests without valid tokens return 401 Unauthorized

Process constraints:
- Workflow: Specify → Plan → Task breakdown → Claude Code execution
- No implementation details inside specs
- No future specs generated unless explicitly instructed
- Each artifact written in Markdown
- Scope creep is not permitted after spec approval

Success criteria:
- All basic-level Todo features implemented as a secure web application
- Each user can access only their own tasks
- Frontend, backend, and database operate as independent services
- JWT-based authentication works end-to-end
- Project can be evaluated purely from specs, plans, and generated code
```

## Response snapshot

The constitution file `.specify/memory/constitution.md` has been updated with the core principles, standards, and constraints for the Phase II Multi-User Todo Full-Stack Web Application. The constitution includes:

- Six core principles: Spec-first development, Single responsibility per spec, Explicit contracts, Security by default, Determinism, and Agentic discipline
- Technology standards covering REST APIs, user data isolation, and communication contracts
- Technology constraints specifying Next.js 16+, Python FastAPI, SQLModel, Neon Serverless PostgreSQL, and Better Auth
- Security standards requiring JWT authentication and proper user identity verification
- Process constraints following the Spec → Plan → Tasks → Implementation workflow
- Success criteria for the project

## Outcome

- ✅ Impact: Updated constitution with comprehensive principles for the multi-user todo application project
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with version 1.1.0
- 🔁 Next prompts: Planning and specification of individual features
- 🧠 Reflection: The constitution provides a solid foundation for the project with clear principles and constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Begin feature specification based on the constitution