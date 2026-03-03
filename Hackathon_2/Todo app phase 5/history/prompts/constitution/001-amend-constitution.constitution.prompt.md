---
id: "001"
title: "Amend Constitution"
stage: "constitution"
date_iso: "2026-01-09"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "001-auth-integration"
user: "user"
command: "sp.constitution"
labels: ["constitution", "governance", "policy"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - ".specify/memory/constitution.md"
tests_yaml: []
---

# Constitution Amendment Record

## Prompt Text

```
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

override constitution file
```

## Response Text

Updated the project constitution at `.specify/memory/constitution.md` with the new core principles and standards. The constitution now includes 6 core principles: Spec-first development, Single responsibility per spec, Explicit contracts, Security by default, Determinism, and Agentic discipline. Also included updated technology constraints, security standards, process constraints, and success criteria. The version was incremented from 1.0.0 to 1.1.0 with a sync impact report noting the changes. Dependent templates were checked for alignment.

## Outcome

The constitution was successfully updated with the new principles and standards for the Multi-User Todo Full-Stack Web Application. The file now properly reflects the project's governance and development standards.

## Evaluation

The constitution update was completed successfully with all required elements incorporated and proper versioning maintained.