---
id: 005
title: Phase V Constitution Update
stage: constitution
date: 2026-02-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: unknown
user: user
command: /sp.constitution
labels: ["constitution", "phase-5", "dapr", "microservices", "environment-separation", "resource-constraints"]
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
# Phase V Constitution

## 1. Environment Separation

We define two environments:

| Environment   | Purpose              | Scale Level |
|--------------|----------------------|------------|
| local-dev    | Development & testing | Minimal |
| cloud-prod   | Final deployment      | Full scale |

Local environment does NOT need to match production scale.

---

## 2. Resource Constraints (Local)

Local development must:

- Use ≤ 2.5GB RAM
- Use ≤ 10GB cluster storage
- Use single-node Kubernetes
- Disable HA mode
- Disable persistent volumes unless required
- Use single replicas for all services

---

## 3. Architecture Rules

- All services communicate through Dapr
- No direct Kafka client in production branch
- Configuration must support:
  - values-dev.yaml
  - values-prod.yaml
- No hardcoded production values

---

## 4. Messaging Rules

Local:
- Redpanda replicas: 1
- Persistence: disabled
- Low memory limits
- Dapr single replica

Production:
- Redpanda scalable
- Persistence enabled
- Dapr HA enabled

---

## 5. CI/CD Rule

- main branch must be cloud deployable
- dev branch optimized for local resources
```

## Response snapshot

Updated constitution from version 3.0.0 to 4.0.0 (MAJOR version bump) to reflect Phase V architectural evolution to Dapr-based microservices with environment separation.

**Key Changes:**
- Added Principle XIII: Environment separation (local-dev vs cloud-prod)
- Added Principle XIV: Dapr-first communication (no direct Kafka clients)
- Added Principle XV: Resource-aware local development (≤2.5GB RAM, ≤10GB storage)
- Updated technology stack to include Dapr service mesh and Redpanda messaging
- Added environment-specific constraints section with detailed resource limits
- Added Dapr Standards and Messaging Standards sections
- Updated branch strategy: `dev` for local-dev, `main` for cloud-prod
- Updated Definition of Done to include Environment, Dapr, and Resource reviews

**Rationale for MAJOR version:**
This represents a backward-incompatible architectural shift from direct service communication to Dapr-mediated communication, and introduces mandatory environment separation that affects all deployment workflows.

## Outcome

- ✅ Impact: Constitution updated to Phase V with 3 new principles governing Dapr microservices architecture, environment separation, and resource-aware development
- 🧪 Tests: N/A (constitution is a governance document)
- 📁 Files: 1 file modified (.specify/memory/constitution.md)
- 🔁 Next prompts: Create feature spec for Dapr integration, plan Redpanda messaging implementation, define environment-specific Helm values
- 🧠 Reflection: The constitution now explicitly supports dual-environment development (local minimal vs cloud scalable), which is critical for developer accessibility and cost-effective iteration

## Evaluation notes (flywheel)

- Failure modes observed: None - constitution update completed successfully with all placeholders filled
- Graders run and results (PASS/FAIL): PASS - All template validations passed, no unresolved placeholders, version incremented correctly, sync impact report generated
- Prompt variant (if applicable): Standard /sp.constitution workflow
- Next experiment (smallest change to try): Validate that existing templates (spec, plan, tasks) align with new Dapr and environment separation principles
