<!--
Sync Impact Report:
- Version: 2.0.0 → 3.0.0 (MAJOR - Architectural evolution to Cloud Native deployment)
- Ratification Date: 2026-01-07 (original)
- Last Amended: 2026-02-05
- Principles Modified:
  * Principle I: Spec-first development (unchanged - still applies)
  * Principle II: Single responsibility per spec (unchanged - still applies)
  * Principle III: Explicit contracts (unchanged - still applies)
  * Principle IV: Security by default (unchanged - still applies)
  * Principle V: Determinism (unchanged - still applies)
  * Principle VI: Agentic discipline (unchanged - still applies)
  * Principle VII: Stateless AI interactions (unchanged - still applies)
  * Principle VIII: Conversation persistence (unchanged - still applies)
  * Principle IX: User data isolation in AI context (unchanged - still applies)
  * NEW Principle X: Container immutability (added for Phase 4)
  * NEW Principle XI: Configuration externalization (added for Phase 4)
  * NEW Principle XII: Infrastructure as Code (added for Phase 4)
- Sections Updated:
  * Project name: "Todo AI Chatbot (Phase 3)" → "Cloud Native Todo Chatbot (Phase 4)"
  * Architecture Overview: Added Container Layer (Docker) and Orchestration Layer (Kubernetes)
  * Technology Stack: Added Docker, Kubernetes (Minikube), Helm, kubectl-ai, kagent, Gordon
  * Technology Standards: Added containerization requirements, Helm chart mandates, AI ops tooling
  * Deployment Standards: Added multi-stage build requirements, stateless container mandate, secrets injection
  * Success Criteria: Updated to reflect cloud-native deployment capabilities
- Templates Status:
  ✅ spec-template.md - Aligned (spec-first requirement matches principle I)
  ✅ plan-template.md - Aligned (constitution check gate present)
  ✅ tasks-template.md - Aligned (task organization matches principles, may need deployment phase examples)
  ⚠ commands/*.md - No command files found in .specify/templates/commands/, no updates needed
- Follow-up: None - all placeholders filled, all principles defined
-->

# Phase IV – Cloud Native Todo Chatbot Constitution

## Purpose

This constitution defines the non-negotiable principles, constraints, and workflow rules governing the Cloud Native Todo Chatbot application, which extends the Phase III AI-powered chatbot with containerization and Kubernetes orchestration for local deployment.

It applies uniformly to:
- Frontend (Next.js + OpenAI ChatKit)
- Backend (Python FastAPI)
- AI Layer (OpenAI Agents SDK + Gemini)
- Tool Layer (MCP Server)
- ORM (SQLModel)
- Database (Neon Serverless PostgreSQL - External Service)
- Authentication (Better Auth)
- Container Layer (Docker)
- Orchestration Layer (Kubernetes via Minikube)
- Deployment (Helm Charts)

## Core Principles

### I. Spec-first development

**Rule**: No planning or implementation may begin without an approved `spec.md`. All work MUST follow the sequence: spec → plan → tasks → implementation.

**Rationale**: Specs are the source of truth. Code must conform to specs, not vice versa. This ensures alignment between business requirements and technical implementation, prevents scope creep, and provides clear acceptance criteria before any code is written.

### II. Single responsibility per spec

**Rule**: Each spec must have a single, well-defined responsibility. No overlapping domains or functionalities between different specs are permitted.

**Rationale**: Clear separation of responsibilities prevents confusion, reduces coupling between different parts of the system, and enables focused development and testing. This principle ensures that each component has a single, well-defined purpose.

### III. Explicit contracts

**Rule**: All cross-boundary behavior must be explicitly defined in specs, not assumed. All interactions between components must have clearly defined contracts.

**Rationale**: Assumptions lead to integration issues and unexpected behavior. By requiring explicit contracts, we ensure that all interactions are well-understood, documented, and tested. This prevents scope creep and ensures predictable system behavior.

### IV. Security by default

**Rule**: All APIs must be authenticated unless explicitly stated otherwise. Security must be built into the system from the ground up.

**Rationale**: Security cannot be retrofitted effectively. By mandating authentication by default, we ensure that all endpoints are secure from the start, preventing unauthorized access and protecting user data. This principle ensures consistent security posture across all domains.

### V. Determinism

**Rule**: The same spec and prompts must produce equivalent outputs. All processes must be reproducible and deterministic.

**Rationale**: Deterministic processes ensure consistency, enable reliable testing, and allow for predictable outcomes. This principle prevents random behavior and ensures that the development process is repeatable and reliable.

### VI. Agentic discipline

**Rule**: All code must be generated via Claude Code; no manual coding is permitted. All implementation must follow the agentic workflow.

**Rationale**: Agentic discipline ensures consistency, maintains the integrity of the development process, and prevents deviations from the established workflow. This principle ensures that all code is generated through the proper channels and follows established patterns.

### VII. Stateless AI interactions

**Rule**: The Chat API endpoint (`POST /api/chat`) MUST be stateless. Each request must be self-contained with all necessary context provided explicitly.

**Rationale**: Stateless design ensures scalability, simplifies debugging, and prevents state-related bugs. The AI agent should not rely on in-memory state between requests. All conversation context must be retrieved from the database and passed explicitly to the agent on each request.

### VIII. Conversation persistence

**Rule**: All conversation messages (user prompts and assistant responses) MUST be persisted to the database before and after agent processing.

**Rationale**: Persistence ensures conversation history is never lost, enables audit trails, supports conversation replay for debugging, and allows users to resume conversations across sessions. This is critical for a production chatbot system.

### IX. User data isolation in AI context

**Rule**: AI agents MUST strictly filter all operations by `user_id`. An agent must never access, modify, or reveal data belonging to another user, even when explicitly prompted.

**Rationale**: User data isolation is a security requirement. The AI layer must enforce the same data isolation guarantees as the API layer. This prevents prompt injection attacks where a malicious user tries to trick the agent into accessing another user's data.

### X. Container immutability

**Rule**: Containers MUST be stateless and immutable. No local file storage is permitted within containers. All application state MUST persist to the external Neon database.

**Rationale**: Immutable containers enable reliable deployments, simplify rollbacks, and ensure consistency across environments. Stateless design allows horizontal scaling and prevents data loss during pod restarts. This is a fundamental requirement for cloud-native applications.

### XI. Configuration externalization

**Rule**: All secrets (API keys, database URLs, JWT secrets) MUST be injected via Kubernetes Secrets. Secrets MUST NEVER be hardcoded in Dockerfiles, source code, or committed to version control.

**Rationale**: Externalizing configuration separates concerns between code and environment-specific settings. Using Kubernetes Secrets ensures secure storage and injection of sensitive data, prevents accidental exposure in version control, and enables environment-specific configuration without code changes.

### XII. Infrastructure as Code

**Rule**: All Kubernetes resources MUST be defined via Helm Charts. Manual `kubectl apply` commands are prohibited for production deployments. Infrastructure definitions must be versioned and reproducible.

**Rationale**: Infrastructure as Code ensures reproducibility, enables version control of infrastructure changes, simplifies rollbacks, and provides a single source of truth for deployment configuration. Helm charts enable templating and environment-specific overrides while maintaining consistency.

## Key Standards

### Technology Standards
- All functionality must trace back to an approved spec
- REST APIs must be stateless and JWT-protected
- User data isolation must be enforced at the query level AND in AI tool definitions
- Frontend and backend communicate only via defined API contracts
- Environment-based configuration for all secrets (including Gemini API keys)
- No hidden coupling between frontend and backend implementations
- MCP tools must have strict Pydantic schemas for inputs and outputs
- All database and API I/O must use `async/await`
- AI agents must degrade gracefully with user-friendly error messages
- Docker images must use multi-stage builds to minimize size
- All Kubernetes resources must be managed via Helm charts
- AI ops tools (kubectl-ai, kagent, Gordon) must be consulted for manifest generation and optimization

### Technology Constraints
- Frontend: Next.js 16+ with App Router + OpenAI ChatKit
- Backend: Python FastAPI (Python 3.10+)
- AI Layer: OpenAI Agents SDK configured with Gemini API as LLM provider
- Tool Layer: Model Context Protocol (MCP) Server using official Python SDK
- ORM: SQLModel
- Database: Neon Serverless PostgreSQL (External Service)
- Authentication: Better Auth (JWT-based)
- Auth transport: Authorization: Bearer <token>
- Container Runtime: Docker (via Docker Desktop)
- Orchestration: Kubernetes (Minikube for local deployment)
- Deployment: Helm Charts (single umbrella chart or separate charts per service)
- AI Ops: kubectl-ai (manifest generation), kagent (cluster analysis), Gordon (Dockerfile optimization)

### Security Standards
- All endpoints (including chat) require a valid JWT after authentication is introduced
- Backend must verify JWT signature and expiry
- User identity must be derived exclusively from JWT claims
- URL user_id must match authenticated user identity
- Requests without valid tokens return 401 Unauthorized
- AI agents must validate user_id on every tool call
- MCP tools must never bypass user_id filtering
- All secrets must be stored in Kubernetes Secrets, never in Dockerfiles or source code
- Secrets must be injected as environment variables at runtime
- Container images must not contain hardcoded credentials

### AI/Agent Standards
- Chat endpoint must be stateless (no in-memory conversation state)
- Conversation history must be loaded from database on each request
- User messages must be persisted BEFORE agent processing
- Assistant responses must be persisted AFTER agent processing
- MCP tools must use strict Pydantic models for type safety
- All MCP tool operations must be async
- Agent errors must return user-friendly messages, not stack traces
- Tools must validate user_id matches authenticated user before any operation

### Deployment Standards
- Containers must be stateless and immutable
- No local file storage within containers
- All application state persists to Neon database (external service)
- Docker images must use multi-stage builds for optimization
- Backend service exposed internally via ClusterIP
- Frontend service exposed locally via LoadBalancer (minikube tunnel) or NodePort
- All Kubernetes resources defined via Helm charts
- Helm charts must support environment-specific value overrides
- Gordon (Docker AI) must be consulted for Dockerfile optimization
- kubectl-ai must be used for Kubernetes manifest generation
- kagent must be used for cluster analysis and troubleshooting

### Process Constraints
- Workflow: Specify → Plan → Task breakdown → Claude Code execution
- No implementation details inside specs
- No future specs generated unless explicitly instructed
- Each artifact written in Markdown
- Scope creep is not permitted after spec approval

## Success Criteria

- Users can manage tasks via natural language conversation
- AI agent correctly interprets user intent (add, list, update, delete tasks)
- Each user can access only their own tasks through the AI interface
- Conversation history is persisted and retrievable
- Frontend (ChatKit), backend (FastAPI), AI layer (Agents SDK), and tool layer (MCP) operate as independent, well-defined components
- JWT-based authentication works end-to-end for chat endpoints
- Application runs in local Kubernetes cluster (Minikube)
- Frontend and backend are containerized with optimized Docker images
- All services are deployed via Helm charts
- Application connects to external Neon database successfully
- Secrets are injected via Kubernetes Secrets
- Services are accessible locally (frontend via LoadBalancer/NodePort, backend via ClusterIP)
- Containers are stateless and can be restarted without data loss
- Project can be evaluated purely from specs, plans, and generated code

## Definition of Done (Global)

A domain is considered complete only when ALL of the following criteria are met:

- [ ] **Agents Defined**: All agents for the domain are explicitly documented with clear responsibilities
- [ ] **Skills Defined**: All skills are documented with inputs, outputs, and usage examples
- [ ] **Spec Approved**: `spec.md` exists, is complete, and has been reviewed and approved
- [ ] **Plan Derived**: `plan.md` exists and is directly traceable to `spec.md`
- [ ] **Tasks Derived**: `tasks.md` exists and is directly traceable to `plan.md`
- [ ] **Implementation Complete**: All tasks in `tasks.md` are marked as complete
- [ ] **Tests Pass**: All tests (if specified) pass successfully
- [ ] **Security Review**: Security requirements from Principles IV, VII, VIII, IX, XI and Security Standards are verified
- [ ] **AI Safety Review**: User data isolation in AI context (Principle IX) is verified with test cases
- [ ] **Container Review**: Container immutability (Principle X) is verified - containers are stateless
- [ ] **Secrets Review**: Configuration externalization (Principle XI) is verified - no hardcoded secrets
- [ ] **IaC Review**: Infrastructure as Code (Principle XII) is verified - all resources in Helm charts
- [ ] **Documentation Updated**: All relevant documentation reflects the implemented changes

**Enforcement**: No domain may be marked as "done" or moved to production without satisfying all criteria. Partial completion must be explicitly documented with remaining items tracked.

## Governance

### Amendment Process

1. **Proposal**: Any team member may propose a constitutional amendment by creating a document describing:
   - The principle/rule to be added, modified, or removed
   - Rationale for the change
   - Impact analysis on existing work
   - Migration plan if existing code/specs are affected

2. **Review**: Amendments must be reviewed by the project lead and at least one other team member

3. **Approval**: Amendments require explicit approval before taking effect

4. **Documentation**: Approved amendments must:
   - Update this constitution file
   - Increment the version number appropriately (MAJOR.MINOR.PATCH)
   - Update the "Last Amended" date
   - Create a Sync Impact Report (as HTML comment at top of file)
   - Update all dependent templates and documentation

### Versioning Policy

This constitution follows semantic versioning:

- **MAJOR**: Backward-incompatible changes (principle removal, redefinition of core rules, architectural shifts)
- **MINOR**: Backward-compatible additions (new principles, expanded guidance)
- **PATCH**: Clarifications, wording improvements, typo fixes

### Compliance

- All pull requests and code reviews MUST verify compliance with this constitution
- Any deviation from constitutional principles MUST be explicitly justified and documented
- Complexity that violates constitutional principles must be tracked in the "Complexity Tracking" section of `plan.md`
- The constitution supersedes all other practices, guidelines, and preferences

### Enforcement

- Automated checks should be implemented where possible (e.g., linting rules, CI/CD gates)
- Manual review is required for principles that cannot be automatically verified
- Non-compliance discovered after merge must be addressed immediately or tracked as technical debt with a remediation plan

---

**Version**: 3.0.0 | **Ratified**: 2026-01-07 | **Last Amended**: 2026-02-05
