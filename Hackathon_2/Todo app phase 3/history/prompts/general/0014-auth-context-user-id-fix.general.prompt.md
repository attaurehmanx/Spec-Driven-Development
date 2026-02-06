---
id: 0014
title: Auth Context User ID Fix
stage: general
date: 2026-01-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 1-task-api-persistence
user: ATTAUREHMAN
command: sp.phr
labels: ["authentication", "user-id", "jwt", "context"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - Z:\phase 2\frontend\nextjs\src\contexts\auth-context.tsx
tests:
 - none
---

## Prompt

Fixed user ID extraction in auth context to properly decode JWT token

## Response snapshot

Successfully updated auth context to properly extract user ID from JWT token payload instead of using hardcoded 'mock-user-id'. Updated login and register functions to decode the JWT token and extract the 'sub' field as the user ID, ensuring it matches the backend validation.

## Outcome

- ✅ Impact: Fixed user ID matching between frontend and backend validation
- 🧪 Tests: none
- 📁 Files: Updated auth-context.tsx to properly decode JWT tokens
- 🔁 Next prompts: none
- 🧠 Reflection: JWT token payload must match backend validation requirements

## Evaluation notes (flywheel)

- Failure modes observed: Hardcoded mock user ID not matching JWT sub field
- Graders run and results: none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Test with actual backend authentication