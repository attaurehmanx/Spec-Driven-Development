---
id: 0013
title: API Client Double getInstance Fix
stage: general
date: 2026-01-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 1-task-api-persistence
user: ATTAUREHMAN
command: sp.phr
labels: ["api", "client", "singleton", "getInstance", "error"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - Z:\phase 2\frontend\nextjs\src\services\task-service.ts
tests:
 - none
---

## Prompt

Fixed double getInstance call in task service

## Response snapshot

Successfully fixed the error caused by accidental double getInstance() calls in task service. The error 'TypeError: ...getInstance(...).getInstance is not a function' was resolved by removing the duplicate getInstance() calls.

## Outcome

- ✅ Impact: Fixed API client singleton usage with single getInstance call
- 🧪 Tests: none
- 📁 Files: Updated task-service.ts to use correct getInstance pattern
- 🔁 Next prompts: none
- 🧠 Reflection: Careful replacement is needed to avoid unintended duplicate method calls

## Evaluation notes (flywheel)

- Failure modes observed: Accidental duplicate method calls during global replacement
- Graders run and results: none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Verify task API calls work correctly