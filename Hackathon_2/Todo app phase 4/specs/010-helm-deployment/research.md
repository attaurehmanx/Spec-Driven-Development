# Research: Kubernetes Deployment with Helm

**Feature**: 010-helm-deployment
**Date**: 2026-02-08
**Purpose**: Document research findings, decisions, and rationale for Helm chart implementation

---

## 1. Helm Chart Best Practices

### Research Findings

**Standard Helm Chart Structure**:
- `Chart.yaml`: Metadata (name, version, description, type, appVersion)
- `values.yaml`: Default configuration values with comments
- `templates/`: Kubernetes resource templates with Go templating
- `templates/_helpers.tpl`: Reusable template functions for labels, names, selectors
- `.helmignore`: Files to exclude from chart package (similar to .gitignore)
- `charts/`: Subdirectory for chart dependencies (not needed for this feature)

**Naming Conventions**:
- Chart name: lowercase, hyphen-separated (e.g., `todo-chatbot`)
- Resource names: Use `{{ include "chart.fullname" . }}` helper for consistency
- Template files: Descriptive names matching resource type (e.g., `deployment-backend.yaml`)

**Template Helper Patterns**:
```yaml
# _helpers.tpl standard patterns
{{- define "todo-chatbot.name" -}}
{{- default .Chart.Name .Values.nameOverride | trunc 63 | trimSuffix "-" }}
{{- end }}

{{- define "todo-chatbot.fullname" -}}
{{- if .Values.fullnameOverride }}
{{- .Values.fullnameOverride | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- $name := default .Chart.Name .Values.nameOverride }}
{{- printf "%s-%s" .Release.Name $name | trunc 63 | trimSuffix "-" }}
{{- end }}
{{- end }}

{{- define "todo-chatbot.labels" -}}
helm.sh/chart: {{ include "todo-chatbot.chart" . }}
{{ include "todo-chatbot.selectorLabels" . }}
app.kubernetes.io/version: {{ .Chart.AppVersion | quote }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
{{- end }}

{{- define "todo-chatbot.selectorLabels" -}}
app.kubernetes.io/name: {{ include "todo-chatbot.name" . }}
app.kubernetes.io/instance: {{ .Release.Name }}
{{- end }}
```

**values.yaml Organization**:
- Group related configuration (e.g., all backend settings together)
- Use nested structures for clarity (e.g., `backend.image.repository`)
- Provide sensible defaults
- Add comments explaining each value
- Avoid storing secrets in values.yaml (use Kubernetes Secrets instead)

**Chart Versioning**:
- Follow semantic versioning (MAJOR.MINOR.PATCH)
- Increment PATCH for bug fixes
- Increment MINOR for backward-compatible features
- Increment MAJOR for breaking changes
- `appVersion` tracks application version separately from chart version

### Decision

**Adopted Approach**: Standard Helm chart structure with single umbrella chart for both frontend and backend services.

**Rationale**:
- Single chart simplifies dependency management (frontend depends on backend service)
- Coordinated deployments ensure both services are deployed together
- Easier to version and maintain than separate charts for a two-service application
- Standard structure ensures compatibility with Helm tooling and community expectations

**Alternatives Considered**:
- **Separate charts per service**: Rejected because it adds complexity for managing dependencies and coordinated deployments. Would require a parent chart or manual coordination.
- **Kustomize instead of Helm**: Rejected because Helm provides better templating, versioning, and rollback capabilities. Kustomize is better for simple overlays but less powerful for complex configurations.

---

## 2. Kubernetes Secrets Management

### Research Findings

**Best Practices for Kubernetes Secrets**:
- Store sensitive data in Kubernetes Secrets (not ConfigMaps or values.yaml)
- Use `Opaque` type for generic secrets (default)
- Base64 encoding is required but NOT encryption (Kubernetes handles encryption at rest if configured)
- Never commit secrets to version control
- Use external secret management tools (e.g., Sealed Secrets, External Secrets Operator) for production
- For local development, manual `kubectl create secret` is acceptable

**Base64 Encoding**:
- Kubernetes requires Base64 encoding for secret data
- Encoding is NOT encryption - secrets are still visible to anyone with cluster access
- Use `echo -n "value" | base64` to encode values
- Use `kubectl get secret <name> -o jsonpath='{.data.key}' | base64 -d` to decode

**Secret Injection Patterns**:

**Option 1: Environment Variables** (Recommended for this feature)
```yaml
env:
  - name: DATABASE_URL
    valueFrom:
      secretKeyRef:
        name: todo-chatbot-secrets
        key: DATABASE_URL
```
Pros: Simple, widely supported, works with all applications
Cons: Secrets visible in pod spec, environment variables

**Option 2: Volume Mounts**
```yaml
volumeMounts:
  - name: secrets
    mountPath: /etc/secrets
    readOnly: true
volumes:
  - name: secrets
    secret:
      secretName: todo-chatbot-secrets
```
Pros: More secure (not in environment), supports file-based secrets
Cons: Application must read from files, more complex configuration

**Secret Rotation**:
- Update secret with `kubectl apply` or `kubectl edit`
- Pods must be restarted to pick up new secret values (environment variables)
- Volume-mounted secrets are automatically updated (with propagation delay)
- Use rolling updates to minimize downtime during rotation

### Decision

**Adopted Approach**: Environment variable injection from Kubernetes Secrets, created manually via `kubectl create secret`.

**Rationale**:
- Environment variables are the standard pattern for FastAPI and Next.js applications
- Simple to implement and understand
- No application code changes required
- Secrets are created outside of Helm chart (not stored in values.yaml)
- Manual creation is acceptable for local development (Minikube)

**Alternatives Considered**:
- **Volume mounts**: Rejected because it requires application changes to read from files. Current applications expect environment variables.
- **Secrets in values.yaml**: Rejected due to security risk of committing secrets to version control.
- **External Secrets Operator**: Rejected as overkill for local development. Can be added later for production.

---

## 3. Service Networking Patterns

### Research Findings

**Service Types**:

**ClusterIP** (Default):
- Internal-only access within cluster
- Assigned a cluster-internal IP address
- Services can communicate via DNS (e.g., `todo-chatbot-backend.default.svc.cluster.local`)
- Best for backend services that should not be exposed externally

**NodePort**:
- Exposes service on each node's IP at a static port (30000-32767 range)
- Accessible from outside cluster via `<NodeIP>:<NodePort>`
- Minikube provides `minikube service <name>` command for easy access
- Good for local development and testing

**LoadBalancer**:
- Provisions external load balancer (cloud provider specific)
- Minikube supports LoadBalancer via `minikube tunnel` command
- Provides external IP address for accessing service
- Best for production external access

**Minikube Tunnel**:
- `minikube tunnel` runs as a process that creates network routes
- Assigns external IPs to LoadBalancer services
- Requires admin/sudo privileges
- Must be kept running while accessing services

**Service Discovery and DNS**:
- Kubernetes DNS automatically creates records for services
- Format: `<service-name>.<namespace>.svc.cluster.local`
- Short form within same namespace: `<service-name>`
- Frontend can access backend via `http://todo-chatbot-backend:8000`

**Health Check Probe Configurations**:

**Liveness Probe**:
- Determines if container is alive and should be restarted
- Failure triggers container restart
- Use for detecting deadlocks or unrecoverable errors
- Example: HTTP GET /health returning 200

**Readiness Probe**:
- Determines if container is ready to accept traffic
- Failure removes pod from service endpoints (no traffic sent)
- Use for detecting temporary unavailability (e.g., loading data)
- Example: HTTP GET /health returning 200

**Startup Probe** (Optional):
- Determines if application has started
- Disables liveness/readiness checks until startup succeeds
- Use for slow-starting applications
- Not needed for this feature (fast startup)

**Probe Configuration**:
```yaml
livenessProbe:
  httpGet:
    path: /health
    port: 8000
  initialDelaySeconds: 10  # Wait before first check
  periodSeconds: 10        # Check every 10 seconds
  timeoutSeconds: 5        # Timeout for each check
  failureThreshold: 3      # Restart after 3 failures

readinessProbe:
  httpGet:
    path: /health
    port: 8000
  initialDelaySeconds: 5   # Wait before first check
  periodSeconds: 5         # Check every 5 seconds
  timeoutSeconds: 3        # Timeout for each check
  failureThreshold: 3      # Remove from service after 3 failures
```

### Decision

**Adopted Approach**:
- Backend: ClusterIP service (internal-only)
- Frontend: LoadBalancer service (external access via minikube tunnel)
- Both services: HTTP liveness and readiness probes
- Service discovery: Frontend accesses backend via service DNS name

**Rationale**:
- ClusterIP for backend ensures it's not exposed externally (security)
- LoadBalancer for frontend provides clean external access pattern
- Minikube tunnel is well-documented and easy to use
- Health probes enable automatic restart and traffic management
- DNS-based service discovery is standard Kubernetes pattern

**Alternatives Considered**:
- **NodePort for frontend**: Rejected because LoadBalancer is cleaner (no port range restrictions). NodePort can be fallback if tunnel issues occur.
- **No health probes**: Rejected because probes are essential for production reliability and automatic recovery.
- **Ingress for frontend**: Rejected as overkill for single frontend service. Ingress is better for multiple services with path-based routing.

---

## 4. Deployment Strategies

### Research Findings

**Rolling Update Strategy** (Default):
- Gradually replaces old pods with new pods
- Ensures zero downtime during updates
- Configurable via `maxSurge` and `maxUnavailable`
- Example: `maxSurge: 1, maxUnavailable: 0` means one extra pod during update, no pods unavailable

**Resource Requests and Limits**:
- **Requests**: Minimum resources guaranteed to container (used for scheduling)
- **Limits**: Maximum resources container can use (enforced by kubelet)
- Best practice: Set both to prevent resource starvation and overcommitment

**Resource Sizing Guidelines**:
- Start conservative, monitor actual usage, adjust based on metrics
- Backend (FastAPI): 256Mi memory, 250m CPU (requests), 512Mi memory, 500m CPU (limits)
- Frontend (Next.js): 128Mi memory, 100m CPU (requests), 256Mi memory, 200m CPU (limits)
- CPU: 1 CPU = 1000m (millicores), 250m = 0.25 CPU
- Memory: Mi = Mebibytes (1024-based), M = Megabytes (1000-based)

**Horizontal Pod Autoscaler (HPA)**:
- Automatically scales replicas based on CPU/memory utilization
- Requires metrics-server installed in cluster
- Example: Scale from 1 to 5 replicas when CPU > 80%
- Not implemented in initial version (manual scaling via Helm upgrade)

**Pod Disruption Budgets (PDB)**:
- Ensures minimum number of pods available during voluntary disruptions
- Example: `minAvailable: 1` ensures at least one pod always running
- Not needed for single-replica deployments
- Can be added later when scaling to multiple replicas

### Decision

**Adopted Approach**:
- Rolling update strategy with default settings (maxSurge: 25%, maxUnavailable: 25%)
- Conservative resource requests and limits (configurable via values.yaml)
- Manual scaling via Helm upgrade (no HPA in initial version)
- No PDB for initial single-replica deployment

**Rationale**:
- Rolling updates provide zero-downtime deployments out of the box
- Conservative resource limits prevent resource exhaustion
- Manual scaling is sufficient for initial deployment (HPA can be added later)
- Single-replica deployment doesn't need PDB (no disruption concerns)

**Alternatives Considered**:
- **Recreate strategy**: Rejected because it causes downtime (all old pods terminated before new pods start)
- **Blue-green deployment**: Rejected as overkill for initial version. Requires more complex setup and resource duplication.
- **HPA from start**: Rejected because it adds complexity and requires metrics-server. Can be added later when scaling needs are clear.

---

## 5. AI Ops Tooling Integration

### Research Findings

**kubectl-ai**:
- AI-powered kubectl assistant for manifest generation and validation
- Usage: `kubectl-ai "create a deployment for nginx with 3 replicas"`
- Generates Kubernetes YAML manifests from natural language
- Can be used to validate and optimize existing manifests
- Useful for learning Kubernetes patterns and quick prototyping

**kagent**:
- AI-powered cluster analysis and troubleshooting tool
- Usage: `kagent analyze cluster` or `kagent diagnose pod <name>`
- Provides insights into cluster health, resource usage, and issues
- Can suggest fixes for common problems
- Useful for debugging deployment issues

**Gordon** (Docker AI):
- AI-powered Dockerfile optimization tool
- Already applied in Feature 009 (Containerization)
- Suggests improvements for image size, security, and build performance
- Not directly used in Helm chart creation but ensures optimized base images

### Decision

**Adopted Approach**:
- Document kubectl-ai and kagent as optional tools for manifest generation and troubleshooting
- Include example prompts in quickstart.md for common operations
- Do not require these tools for basic deployment (Helm charts are manually created)
- Recommend using these tools for learning and optimization

**Rationale**:
- AI ops tools are helpful but not required for basic Helm chart creation
- Manual chart creation ensures understanding of Kubernetes concepts
- Tools can be used for validation and optimization after initial implementation
- Documentation provides guidance for users who want to leverage AI assistance

**Alternatives Considered**:
- **Require kubectl-ai for all manifest generation**: Rejected because it adds dependency and may not be available in all environments
- **Skip AI ops tools entirely**: Rejected because they provide value for learning and troubleshooting

---

## Summary of Key Decisions

| Topic | Decision | Rationale |
|-------|----------|-----------|
| Chart Structure | Single umbrella chart | Simplifies dependency management for two-service app |
| Secret Management | Environment variables from Kubernetes Secrets | Standard pattern, no app changes required |
| Backend Service | ClusterIP (internal-only) | Security - backend should not be exposed externally |
| Frontend Service | LoadBalancer (via minikube tunnel) | Clean external access pattern for local development |
| Health Probes | HTTP liveness and readiness probes | Automatic restart and traffic management |
| Deployment Strategy | Rolling updates with default settings | Zero-downtime deployments out of the box |
| Resource Limits | Conservative defaults, configurable | Prevents resource exhaustion, tunable based on usage |
| Scaling | Manual via Helm upgrade | Sufficient for initial deployment, HPA can be added later |
| AI Ops Tools | Optional, documented for reference | Helpful but not required for basic deployment |

---

## Next Steps

1. Create data-model.md with detailed Kubernetes resource definitions
2. Create contracts/ directory with Helm chart structure and templates
3. Create quickstart.md with step-by-step deployment instructions
4. Implement Helm chart based on research findings and decisions
5. Validate chart with `helm lint` and `helm template`
6. Deploy to Minikube and verify all success criteria
