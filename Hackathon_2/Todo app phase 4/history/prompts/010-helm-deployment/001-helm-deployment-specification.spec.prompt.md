---
id: 001
title: helm-deployment-specification
stage: spec
date: 2026-02-08
surface: agent
model: claude-opus-4-6
feature: 010-helm-deployment
branch: 010-helm-deployment
user: unknown
command: /sp.specify
labels: ["kubernetes", "helm", "deployment", "devops", "secrets-management"]
links:
  spec: specs/010-helm-deployment/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/010-helm-deployment/spec.md
  - specs/010-helm-deployment/checklists/requirements.md
tests:
  - N/A (specification phase)
---

## Prompt

# Spec 2: Kubernetes Deployment (Helm)

## Context
We will use Helm to manage the deployment complexity.

## Helm Chart Structure
Create a chart named `todo-chatbot` with the following components:

### 1. Secrets (k8s-secrets.yaml)
* Must store:
    * `DATABASE_URL` (From Neon)
    * `GEMINI_API_KEY` (or OpenAI Key)
    * `BETTER_AUTH_SECRET`

### 2. Backend Deployment
* **Replicas:** 1 (Scale to 2 later using kubectl-ai).
* **Image:** `todo-backend:latest` (PullPolicy: Never/IfNotPresent for local).
* **Env Vars:** Map from Secrets.
* **Liveness Probe:** `/health` endpoint.

### 3. Frontend Deployment
* **Replicas:** 1.
* **Image:** `todo-frontend:latest`.
* **Env Vars:** `NEXT_PUBLIC_API_URL` (Must point to the k8s service).

### 4. Networking (Services)
* **Backend Service:** Type `ClusterIP` (Internal only).
* **Frontend Service:** Type `LoadBalancer` (Exposed to localhost).

## AI Ops Workflow
* **Prompt (kubectl-ai):** "Generate a Kubernetes Deployment manifest for a Python FastAPI app that uses env vars from a Secret."
* **Prompt (kubectl-ai):** "Create a Service that exposes port 3000 as a LoadBalancer."

## Response snapshot

Created comprehensive specification for Kubernetes Helm deployment feature (010-helm-deployment) with:

**User Stories (5 prioritized)**:
- P1: Deploy Application to Production Environment
- P1: Manage Sensitive Configuration Securely
- P2: Monitor Application Health
- P3: Scale Application Based on Load
- P2: Access Application from External Network

**Functional Requirements (12)**:
- Deployment of frontend and backend services
- Secure secrets management for credentials and API keys
- Health monitoring with automatic failure detection
- Scaling capabilities for backend services
- Network routing (external frontend, internal backend)
- Zero-downtime configuration updates

**Success Criteria (8 measurable outcomes)**:
- Deployment time < 5 minutes
- 99.9% uptime
- 30-second failure recovery
- 2-minute scaling time
- 3-second page load
- Zero security incidents
- Zero-downtime updates
- 100 concurrent users support

**Validation**: All checklist items passed - specification is complete, unambiguous, and ready for planning phase.

## Outcome

- ✅ Impact: Created production-ready specification for Kubernetes deployment with comprehensive coverage of deployment, security, monitoring, and scaling requirements
- 🧪 Tests: Specification validation checklist passed all 14 criteria (content quality, requirement completeness, feature readiness)
- 📁 Files: Created spec.md (168 lines) and requirements.md checklist (complete validation)
- 🔁 Next prompts: `/sp.plan` to design implementation architecture, `/sp.clarify` if additional requirements emerge
- 🧠 Reflection: Successfully translated technical deployment requirements into business-focused specification with clear priorities, measurable outcomes, and well-defined scope boundaries

## Evaluation notes (flywheel)

- Failure modes observed: None - specification completed without clarification needs by making informed assumptions about deployment standards
- Graders run and results (PASS/FAIL): Specification Quality Checklist - PASS (14/14 criteria met)
- Prompt variant (if applicable): Standard /sp.specify workflow with technical deployment context
- Next experiment (smallest change to try): Consider adding rollback scenarios as additional user story for production safety
