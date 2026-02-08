# Implementation Plan: Kubernetes Deployment with Helm

**Branch**: `010-helm-deployment` | **Date**: 2026-02-08 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/010-helm-deployment/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Deploy the todo-chatbot application (frontend and backend) to a local Kubernetes cluster using Helm charts. The deployment must include secure secrets management for database credentials and API keys, health monitoring with automatic restart capabilities, service networking (internal backend, external frontend), and support for horizontal scaling. The solution uses Helm to manage deployment complexity and follows Infrastructure as Code principles.

## Technical Context

**Language/Version**: Helm 3.x, Kubernetes 1.28+, YAML for manifests
**Primary Dependencies**: Helm CLI, kubectl, Kubernetes cluster (Minikube), Docker images (todo-backend:latest, todo-frontend:latest from Feature 009)
**Storage**: N/A (deployment configuration only - application state persists to external Neon database)
**Testing**: helm lint (chart validation), helm template (manifest generation), kubectl apply --dry-run (resource validation), kubectl get/describe (deployment verification)
**Target Platform**: Kubernetes (Minikube for local development, extensible to production clusters)
**Project Type**: Infrastructure as Code (Helm charts for Kubernetes resources)
**Performance Goals**: Initial deployment < 5 minutes, service scaling < 2 minutes, health check response < 30 seconds
**Constraints**: Containers must be stateless and immutable, all secrets via Kubernetes Secrets (never hardcoded), backend internal-only (ClusterIP), frontend externally accessible (LoadBalancer/NodePort)
**Scale/Scope**: 2 services (frontend, backend), 1 umbrella Helm chart (todo-chatbot), 3 secrets (DATABASE_URL, GEMINI_API_KEY, BETTER_AUTH_SECRET), support for 1-N replicas per service

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle Compliance

✅ **I. Spec-first development**: Approved spec.md exists at `specs/010-helm-deployment/spec.md`

✅ **II. Single responsibility per spec**: This spec focuses exclusively on Kubernetes deployment via Helm - does not overlap with containerization (009), application features, or infrastructure setup

✅ **III. Explicit contracts**: Deployment contracts are explicit:
- Helm chart structure and templates
- Kubernetes resource definitions (Deployments, Services, Secrets)
- Environment variable injection from Secrets to containers
- Service networking (ClusterIP for backend, LoadBalancer for frontend)

✅ **IV. Security by default**:
- All secrets stored in Kubernetes Secrets (never hardcoded)
- Backend service internal-only (ClusterIP)
- JWT authentication enforced at application level (inherited from previous features)

✅ **V. Determinism**: Helm charts are declarative and reproducible - same chart + values produce identical Kubernetes resources

✅ **VI. Agentic discipline**: All Helm chart generation and configuration via Claude Code workflow

✅ **VII. Stateless AI interactions**: N/A (deployment layer - AI statelessness enforced at application level)

✅ **VIII. Conversation persistence**: N/A (deployment layer - persistence enforced at application level)

✅ **IX. User data isolation in AI context**: N/A (deployment layer - isolation enforced at application level)

✅ **X. Container immutability**: Containers are stateless and immutable (no local file storage, all state in external Neon database)

✅ **XI. Configuration externalization**: All secrets injected via Kubernetes Secrets as environment variables (DATABASE_URL, GEMINI_API_KEY, BETTER_AUTH_SECRET)

✅ **XII. Infrastructure as Code**: All Kubernetes resources defined via Helm charts (no manual kubectl apply for production)

### Technology Standards Compliance

✅ All Kubernetes resources managed via Helm charts
✅ Docker images use multi-stage builds (inherited from Feature 009)
✅ Backend service exposed internally via ClusterIP
✅ Frontend service exposed locally via LoadBalancer (minikube tunnel) or NodePort
✅ Helm charts support environment-specific value overrides

### Security Standards Compliance

✅ All secrets stored in Kubernetes Secrets
✅ Secrets injected as environment variables at runtime
✅ Container images do not contain hardcoded credentials
✅ Backend service not exposed externally (ClusterIP only)

### Deployment Standards Compliance

✅ Containers are stateless and immutable
✅ No local file storage within containers
✅ All application state persists to Neon database (external service)
✅ All Kubernetes resources defined via Helm charts
✅ Helm charts support environment-specific value overrides

### Gate Result: ✅ PASS

All constitutional principles and standards are satisfied. No violations to justify.

## Project Structure

### Documentation (this feature)

```text
specs/010-helm-deployment/
├── spec.md              # Feature specification (complete)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - Helm best practices, secrets management patterns
├── data-model.md        # Phase 1 output - Kubernetes resource definitions
├── quickstart.md        # Phase 1 output - Deployment instructions
├── contracts/           # Phase 1 output - Helm chart structure and templates
│   ├── chart-structure.md
│   ├── values-schema.md
│   └── resource-templates.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
helm-charts/
└── todo-chatbot/              # Helm chart root
    ├── Chart.yaml             # Chart metadata (name, version, description)
    ├── values.yaml            # Default configuration values
    ├── templates/             # Kubernetes resource templates
    │   ├── _helpers.tpl       # Template helpers (labels, selectors)
    │   ├── secret.yaml        # Kubernetes Secret for sensitive data
    │   ├── deployment-backend.yaml   # Backend Deployment
    │   ├── service-backend.yaml      # Backend Service (ClusterIP)
    │   ├── deployment-frontend.yaml  # Frontend Deployment
    │   └── service-frontend.yaml     # Frontend Service (LoadBalancer)
    └── .helmignore            # Files to exclude from chart package

# Existing structure (unchanged)
backend/                       # FastAPI application (Feature 009)
frontend-app/                  # Next.js application (Feature 009)
specs/                         # Feature specifications
history/                       # Prompt history records
.specify/                      # SpecKit Plus templates and scripts
```

**Structure Decision**: Created new `helm-charts/todo-chatbot/` directory at repository root to house the Helm chart. This separates infrastructure definitions from application code, follows Helm conventions, and enables independent versioning of deployment configurations. The chart uses a single umbrella structure (not separate charts per service) to simplify dependency management and ensure coordinated deployments.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected. All constitutional principles are satisfied.

---

## Phase 0: Research & Unknowns Resolution

### Research Tasks

1. **Helm Chart Best Practices**
   - Research: Standard Helm chart structure and naming conventions
   - Research: Template helper patterns for labels and selectors
   - Research: values.yaml organization for multi-service charts
   - Research: Chart versioning and dependency management

2. **Kubernetes Secrets Management**
   - Research: Best practices for storing sensitive data in Kubernetes Secrets
   - Research: Base64 encoding requirements and security implications
   - Research: Secret injection patterns (environment variables vs volume mounts)
   - Research: Secret rotation and update strategies

3. **Service Networking Patterns**
   - Research: ClusterIP vs LoadBalancer vs NodePort service types
   - Research: Minikube tunnel for LoadBalancer access on local clusters
   - Research: Service discovery and DNS resolution within cluster
   - Research: Health check probe configurations (liveness, readiness, startup)

4. **Deployment Strategies**
   - Research: Rolling update strategies and zero-downtime deployments
   - Research: Resource requests and limits for containers
   - Research: Horizontal Pod Autoscaler (HPA) configuration for future scaling
   - Research: Pod disruption budgets for high availability

5. **AI Ops Tooling Integration**
   - Research: kubectl-ai usage for manifest generation and validation
   - Research: kagent usage for cluster analysis and troubleshooting
   - Research: Gordon usage for Dockerfile optimization (already applied in Feature 009)

### Unknowns to Resolve

All technical context items are specified. No NEEDS CLARIFICATION markers present.

### Research Output Location

All research findings will be consolidated in `specs/010-helm-deployment/research.md` with decisions, rationale, and alternatives considered for each topic.

---

## Phase 1: Design & Contracts

### Data Model

**Kubernetes Resources** (defined in `data-model.md`):

1. **Secret** (`todo-chatbot-secrets`)
   - Type: Opaque
   - Data fields: DATABASE_URL, GEMINI_API_KEY, BETTER_AUTH_SECRET (Base64 encoded)
   - Namespace: default (or configurable via values)

2. **Deployment** (`todo-chatbot-backend`)
   - Replicas: 1 (configurable via values.yaml)
   - Container: todo-backend:latest
   - Environment variables: Injected from Secret
   - Liveness probe: HTTP GET /health
   - Readiness probe: HTTP GET /health
   - Resource limits: Configurable via values.yaml

3. **Service** (`todo-chatbot-backend`)
   - Type: ClusterIP
   - Port: 8000 (FastAPI default)
   - Selector: app=todo-chatbot, component=backend

4. **Deployment** (`todo-chatbot-frontend`)
   - Replicas: 1 (configurable via values.yaml)
   - Container: todo-frontend:latest
   - Environment variables: NEXT_PUBLIC_API_URL (points to backend service)
   - Liveness probe: HTTP GET / (or /api/health if available)
   - Readiness probe: HTTP GET /
   - Resource limits: Configurable via values.yaml

5. **Service** (`todo-chatbot-frontend`)
   - Type: LoadBalancer (or NodePort for Minikube)
   - Port: 3000 (Next.js default)
   - Selector: app=todo-chatbot, component=frontend

### API Contracts

**Helm Chart Interface** (defined in `contracts/`):

1. **Chart.yaml Contract**
   ```yaml
   apiVersion: v2
   name: todo-chatbot
   description: Helm chart for todo-chatbot application
   type: application
   version: 0.1.0
   appVersion: "1.0"
   ```

2. **values.yaml Contract**
   ```yaml
   # Backend configuration
   backend:
     image:
       repository: todo-backend
       tag: latest
       pullPolicy: IfNotPresent
     replicaCount: 1
     service:
       type: ClusterIP
       port: 8000
     resources:
       requests:
         memory: "256Mi"
         cpu: "250m"
       limits:
         memory: "512Mi"
         cpu: "500m"
     healthCheck:
       path: /health
       initialDelaySeconds: 10
       periodSeconds: 10

   # Frontend configuration
   frontend:
     image:
       repository: todo-frontend
       tag: latest
       pullPolicy: IfNotPresent
     replicaCount: 1
     service:
       type: LoadBalancer  # or NodePort for Minikube
       port: 3000
     resources:
       requests:
         memory: "128Mi"
         cpu: "100m"
       limits:
         memory: "256Mi"
         cpu: "200m"
     healthCheck:
       path: /
       initialDelaySeconds: 15
       periodSeconds: 10

   # Secrets (values provided at deployment time, not in values.yaml)
   secrets:
     databaseUrl: ""  # Set via --set or separate values file
     geminiApiKey: ""
     betterAuthSecret: ""
   ```

3. **Template Contracts**
   - All templates use `{{ include "todo-chatbot.fullname" . }}` for resource naming
   - All templates use `{{ include "todo-chatbot.labels" . }}` for consistent labeling
   - All templates support `.Values` overrides for configuration
   - All templates include namespace support via `.Release.Namespace`

### Quickstart

Deployment instructions will be documented in `quickstart.md`:

1. **Prerequisites**
   - Minikube installed and running
   - kubectl configured to access Minikube cluster
   - Helm 3.x installed
   - Docker images built (todo-backend:latest, todo-frontend:latest)

2. **Deployment Steps**
   ```bash
   # Create secrets (one-time setup)
   kubectl create secret generic todo-chatbot-secrets \
     --from-literal=DATABASE_URL="postgresql://..." \
     --from-literal=GEMINI_API_KEY="..." \
     --from-literal=BETTER_AUTH_SECRET="..."

   # Install Helm chart
   helm install todo-chatbot ./helm-charts/todo-chatbot

   # Verify deployment
   kubectl get pods
   kubectl get services

   # Access frontend (if LoadBalancer)
   minikube tunnel  # Run in separate terminal
   kubectl get service todo-chatbot-frontend  # Get EXTERNAL-IP

   # Access frontend (if NodePort)
   minikube service todo-chatbot-frontend
   ```

3. **Scaling Operations**
   ```bash
   # Scale backend to 2 replicas
   helm upgrade todo-chatbot ./helm-charts/todo-chatbot \
     --set backend.replicaCount=2

   # Or edit values.yaml and upgrade
   helm upgrade todo-chatbot ./helm-charts/todo-chatbot
   ```

4. **Troubleshooting**
   ```bash
   # Check pod status
   kubectl describe pod <pod-name>

   # View logs
   kubectl logs <pod-name>

   # Check service endpoints
   kubectl get endpoints

   # Use kagent for cluster analysis
   kagent analyze cluster
   ```

### Agent Context Update

After Phase 1 design completion, run:
```bash
.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude
```

This will update the Claude-specific context file with:
- Helm chart structure and location
- Kubernetes resource definitions
- Deployment and scaling procedures
- Troubleshooting commands

---

## Phase 2: Task Breakdown

Task breakdown will be generated by the `/sp.tasks` command (not part of `/sp.plan`).

Expected task categories:
1. Helm chart scaffolding and structure setup
2. Secret template creation and configuration
3. Backend deployment and service manifests
4. Frontend deployment and service manifests
5. Values.yaml configuration and documentation
6. Chart validation and testing
7. Deployment verification and troubleshooting

---

## Implementation Notes

### Key Decisions

1. **Single Umbrella Chart**: Using one Helm chart for both frontend and backend simplifies dependency management and ensures coordinated deployments. Alternative (separate charts) rejected because it adds complexity for a two-service application.

2. **LoadBalancer vs NodePort**: Default to LoadBalancer for frontend service, with NodePort as fallback for Minikube environments. LoadBalancer provides cleaner external access but requires `minikube tunnel`.

3. **Secret Management**: Using Kubernetes Secrets with manual creation (kubectl create secret) rather than storing secrets in values.yaml. This prevents accidental exposure in version control.

4. **Health Check Endpoints**: Backend uses `/health` endpoint (FastAPI standard), frontend uses `/` (Next.js root). Both configured with appropriate initial delays to allow startup time.

5. **Resource Limits**: Conservative defaults (256Mi/250m for backend, 128Mi/100m for frontend) with configurable overrides. These can be tuned based on actual usage patterns.

### Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Secrets exposed in values.yaml | High - Security breach | Document manual secret creation, add .helmignore for sensitive files |
| Minikube tunnel required for LoadBalancer | Medium - User friction | Document both LoadBalancer and NodePort options in quickstart |
| Image pull failures (local images) | High - Deployment failure | Set imagePullPolicy: IfNotPresent, document image build requirements |
| Backend not ready before frontend | Medium - Connection errors | Configure readiness probes with appropriate delays, frontend should handle retries |
| Resource limits too restrictive | Medium - Pod eviction | Start with conservative limits, document monitoring and tuning procedures |

### Dependencies

- **Feature 009 (Containerization)**: Must be complete - requires todo-backend:latest and todo-frontend:latest images
- **Minikube**: Must be installed and running
- **Helm 3.x**: Must be installed
- **kubectl**: Must be configured to access Minikube cluster
- **Database credentials**: Must be available for secret creation
- **API keys**: GEMINI_API_KEY and BETTER_AUTH_SECRET must be available

### Success Criteria Mapping

| Success Criterion | Implementation Approach |
|-------------------|------------------------|
| SC-001: Deploy in < 5 minutes | Helm install with pre-built images, optimized resource allocation |
| SC-002: 99.9% uptime | Health checks with automatic restart, readiness probes |
| SC-003: Restart within 30 seconds | Liveness probe with 10s period, Kubernetes automatic restart |
| SC-004: Scale in < 2 minutes | Helm upgrade with replicaCount override, Kubernetes rolling update |
| SC-005: Page load < 3 seconds | LoadBalancer/NodePort for direct access, no additional proxies |
| SC-006: Zero security incidents | Kubernetes Secrets for sensitive data, ClusterIP for backend |
| SC-007: Zero-downtime updates | Rolling update strategy (default), readiness probes |
| SC-008: 100 concurrent users | Resource limits allow multiple replicas, horizontal scaling support |

---

## Next Steps

1. Execute Phase 0: Generate `research.md` with Helm and Kubernetes best practices
2. Execute Phase 1: Generate `data-model.md`, `contracts/`, and `quickstart.md`
3. Update agent context with Helm chart structure and deployment procedures
4. Run `/sp.tasks` to generate detailed task breakdown
5. Execute tasks via Claude Code to implement Helm chart
6. Validate chart with `helm lint` and `helm template`
7. Deploy to Minikube and verify all success criteria
