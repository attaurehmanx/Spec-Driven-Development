---
id: 002
title: Create Advanced Task Features Plan
stage: plan
date: 2026-02-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: advanced-task-features
branch: 010-advanced-task-features
user: user
command: /sp.plan
labels: ["plan", "phased-migration", "dapr", "redpanda", "database-schema", "api-contracts", "recurring-tasks", "reminders"]
links:
  spec: specs/010-advanced-task-features/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/010-advanced-task-features/plan.md
  - specs/010-advanced-task-features/research.md
  - specs/010-advanced-task-features/data-model.md
  - specs/010-advanced-task-features/contracts/tasks-api.md
  - specs/010-advanced-task-features/contracts/events.md
  - specs/010-advanced-task-features/quickstart.md
tests:
  - none
---

## Prompt

```text
/sp.plan
# Spec-Driven Development Plan

## Phase 1 – Model Update
- Add fields to Task:
  - priority (enum)
  - tags (array<string>)
  - due_date (nullable datetime)
  - recurring (enum)
- Update API endpoints
- Update frontend forms
- Implement search/filter/sort in backend

(No Kafka yet)

---

## Phase 2 – Recurring Logic
- When task completed:
  - If recurring != none
  - Auto-create next task instance

(Handled inside backend logic)

---

## Phase 3 – Add Redpanda (Minimal Mode)

Local configuration:
- replicas: 1
- persistence: disabled
- memory limit: 512Mi

Purpose:
- Enable event publishing
- No HA
- No scaling

---

## Phase 4 – Install Dapr (Single Replica)

- dapr init -k
- Apply Kafka pubsub component
- Apply Postgres statestore component

---

## Phase 5 – Refactor Backend

Replace direct Kafka library usage with:

POST http://localhost:3500/v1.0/publish/kafka-pubsub/topic-name

Remove:
- aiokafka
- kafka-python

---

## Phase 6 – Cloud Deployment (Oracle)

- Create OKE cluster
- Enable persistence
- Enable scaling
- Use values-prod.yaml
- Deploy with Helm

Local remains lightweight.
Production runs full scale.
```

## Response snapshot

Created comprehensive 6-phase implementation plan for advanced task management features, incorporating user's phased Dapr migration strategy:

**Phase 1 (Model Update)**: Database schema extensions (priority, tags, due_date, recurring, parent_task_id), API endpoint updates, frontend UI components, search/filter/sort implementation. No messaging infrastructure.

**Phase 2 (Recurring Logic)**: Backend-only recurring task automation with date calculation logic, handles month-end edge cases per spec decision.

**Phase 3 (Add Redpanda)**: Introduce Redpanda in minimal mode (1 replica, no persistence, 512Mi memory) for local development. Temporary direct Kafka publishing with aiokafka.

**Phase 4 (Install Dapr)**: Add Dapr service mesh with single replica, configure pub/sub and state components, inject Dapr sidecars into backend pods.

**Phase 5 (Refactor Backend)**: Replace direct Kafka client with Dapr HTTP API, remove aiokafka dependency, achieve Principle XIV (Dapr-first communication).

**Phase 6 (Cloud Deployment)**: Deploy to Oracle OKE with production configuration (Redpanda 3 replicas with persistence, Dapr HA enabled, scalable resources).

**Supporting Documentation Created**:
- research.md: Technology research (PostgreSQL arrays, full-text search, Redpanda vs Kafka, Dapr pub/sub, performance optimization)
- data-model.md: Complete database schema with SQLModel definitions, Alembic migration script, validation rules
- contracts/tasks-api.md: Extended REST API contracts with 8 endpoints, query parameters, validation rules
- contracts/events.md: Dapr pub/sub event schemas (task.reminder, task.recurring.created)
- quickstart.md: Detailed setup instructions for all 6 phases with testing procedures

**Key Architectural Decisions**:
- PostgreSQL TEXT[] array for tags with GIN index
- ILIKE pattern matching for search (simple, adequate for scale)
- VARCHAR with CHECK constraint for enums (flexibility over PostgreSQL ENUM)
- Redpanda for Kafka-compatible messaging (lower memory footprint)
- APScheduler for background jobs (Phase 3-4), migrate to K8s CronJob (Phase 6)
- Phased migration strategy maintains local resource constraints (≤2.5GB RAM) while enabling production scalability

**Resource Budget Validation**:
- Local development total: ~2.3GB (within 2.5GB constitutional limit)
- Minikube: 1GB, Backend: 256MB, Frontend: 256MB, Redpanda: 512MB, Dapr: 250MB

## Outcome

- ✅ Impact: Complete 6-phase implementation plan created with phased Dapr migration strategy, aligns with Phase V constitution principles
- 🧪 Tests: Testing procedures defined for each phase with validation checklists
- 📁 Files: 6 files created (plan.md, research.md, data-model.md, 2 contract files, quickstart.md)
- 🔁 Next prompts: Run `/sp.tasks` to generate detailed task breakdown, begin Phase 1 implementation
- 🧠 Reflection: Phased approach successfully balances local resource constraints with production scalability goals, maintains constitutional compliance throughout migration

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully with all constitutional gates passed
- Graders run and results (PASS/FAIL): PASS - All constitution checks passed (15/15 principles), resource budget validated (2.3GB < 2.5GB limit)
- Prompt variant (if applicable): User-guided phased migration strategy incorporated into standard /sp.plan workflow
- Next experiment (smallest change to try): Consider creating phase-specific checklists in addition to overall quickstart guide to improve phase validation clarity
