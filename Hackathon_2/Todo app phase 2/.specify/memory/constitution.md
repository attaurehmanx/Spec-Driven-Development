<!--
Sync Impact Report:
- Version: 1.1.0 → 1.1.0 (No functional changes, just PHR creation)
- Ratification Date: 2026-01-07
- Last Amended: 2026-01-11
- Principles Modified: None (review and validation only)
- Sections Updated: None
- Templates Status:
  ✅ spec-template.md - Aligned (spec-first requirement matches principle 1)
  ✅ plan-template.md - Aligned (constitution check gate present)
  ✅ tasks-template.md - Aligned (task organization matches principles)
  ⚠ commands/*.md - No command files found, no updates needed
- Follow-up: None
-->

# Phase II – Multi-User Todo Full-Stack Web Application Constitution

## Purpose

This constitution defines the non-negotiable principles, constraints, and workflow rules governing the entire multi-domain Multi-User Todo Full-Stack Web Application.

It applies uniformly to:
- Frontend
- Backend
- ORM
- Database
- Authentication

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

## Key Standards

### Technology Standards
- All functionality must trace back to an approved spec
- REST APIs must be stateless and JWT-protected
- User data isolation must be enforced at the query level
- Frontend and backend communicate only via defined API contracts
- Environment-based configuration for all secrets
- No hidden coupling between frontend and backend implementations

### Technology Constraints
- Frontend: Next.js 16+ with App Router
- Backend: Python FastAPI
- ORM: SQLModel
- Database: Neon Serverless PostgreSQL
- Authentication: Better Auth (JWT-based)
- Auth transport: Authorization: Bearer <token>

### Security Standards
- All endpoints require a valid JWT after authentication is introduced
- Backend must verify JWT signature and expiry
- User identity must be derived exclusively from JWT claims
- URL user_id must match authenticated user identity
- Requests without valid tokens return 401 Unauthorized

### Process Constraints
- Workflow: Specify → Plan → Task breakdown → Claude Code execution
- No implementation details inside specs
- No future specs generated unless explicitly instructed
- Each artifact written in Markdown
- Scope creep is not permitted after spec approval

## Success Criteria

- All basic-level Todo features implemented as a secure web application
- Each user can access only their own tasks
- Frontend, backend, and database operate as independent services
- JWT-based authentication works end-to-end
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
- [ ] **Security Review**: Security requirements from Principles IV and Security Standards are verified
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

- **MAJOR**: Backward-incompatible changes (principle removal, redefinition of core rules)
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

**Version**: 1.1.0 | **Ratified**: 2026-01-07 | **Last Amended**: 2026-01-11
