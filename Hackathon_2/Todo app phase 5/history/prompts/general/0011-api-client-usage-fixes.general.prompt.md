---
id: 0011
title: API Client Usage Fixes
stage: general
date: 2026-01-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 1-task-api-persistence
user: ATTAUREHMAN
command: sp.phr
labels: ["api", "client", "singleton", "service", "integration"]
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

Fixed API client usage in task service

## Response snapshot

Successfully updated the task service to properly use ApiClient singleton instance via getInstance() method instead of treating it as a direct API object. This resolves the error where ApiClient.default.get was not a function.

## Outcome

- ✅ Impact: Fixed API client usage with proper singleton pattern
- 🧪 Tests: none
- 📁 Files: Updated task-service.ts to use ApiClient.getInstance()
- 🔁 Next prompts: none
- 🧠 Reflection: Proper singleton pattern usage is critical for class-based services

## Evaluation notes (flywheel)

- Failure modes observed: Incorrect singleton usage causing method not found errors
- Graders run and results: none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Test API functionality with backend