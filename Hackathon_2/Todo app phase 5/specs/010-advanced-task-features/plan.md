# Implementation Plan: Advanced Task Management Features

**Branch**: `010-advanced-task-features` | **Date**: 2026-02-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/010-advanced-task-features/spec.md`

## Summary

Extend the existing task management system with advanced organizational features including priorities (high/medium/low), multi-tag support, search/filter/sort capabilities, due dates with reminder events, and recurring task automation. Implementation follows a phased migration strategy: start with direct database operations (Phase 1-2), introduce Redpanda messaging in minimal mode (Phase 3), add Dapr service mesh (Phase 4), refactor to Dapr-first communication (Phase 5), and enable cloud deployment with full scale (Phase 6). This approach maintains local development resource constraints (≤2.5GB RAM) while enabling production scalability.

## Technical Context

**Language/Version**: Python 3.10+ (backend), TypeScript/Next.js 16+ (frontend)
**Primary Dependencies**: FastAPI, SQLModel, Neon PostgreSQL, Next.js, Dapr (Phase 4+), Redpanda (Phase 3+)
**Storage**: Neon Serverless PostgreSQL (external service)
**Testing**: pytest (backend), Jest/React Testing Library (frontend)
**Target Platform**: Kubernetes (Minikube local-dev, OKE cloud-prod)
**Project Type**: Web application (backend + frontend)
**Performance Goals**:
- Search results in <1 second for 100+ tasks
- Filter/sort operations in <1 second
- Recurring task generation within 1 minute of completion
- Reminder events published with <1 minute variance
**Constraints**:
- Local development: ≤2.5GB RAM total, single-node Kubernetes
- Production: Scalable resources, HA enabled
- All inter-service communication via Dapr (Phase 5+)
**Scale/Scope**:
- 10-500 tasks per user typical
- Support 1000+ concurrent users (production)
- 5 user stories (P1-P5) implemented incrementally

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle Compliance

✅ **I. Spec-first development**: Approved spec.md exists at specs/010-advanced-task-features/spec.md

✅ **II. Single responsibility per spec**: This spec focuses solely on advanced task management features (priorities, tags, search, filter, sort, recurring, due dates)

✅ **III. Explicit contracts**: API contracts will be defined in contracts/ directory (Phase 1)

✅ **IV. Security by default**: All endpoints will require JWT authentication; user_id filtering enforced

✅ **V. Determinism**: Same spec produces equivalent outputs; all processes reproducible

✅ **VI. Agentic discipline**: All code generated via Claude Code

✅ **VII. Stateless AI interactions**: Not applicable (no AI agent changes in this feature)

✅ **VIII. Conversation persistence**: Not applicable (no conversation changes in this feature)

✅ **IX. User data isolation in AI context**: Not applicable (no AI agent changes in this feature)

✅ **X. Container immutability**: Containers remain stateless; all state in Neon database

✅ **XI. Configuration externalization**: Secrets via Kubernetes Secrets; environment-specific values files

✅ **XII. Infrastructure as Code**: All resources defined via Helm charts (values-dev.yaml, values-prod.yaml)

✅ **XIII. Environment separation**: Phased approach explicitly supports local-dev (minimal) and cloud-prod (full scale)

✅ **XIV. Dapr-first communication**: Achieved in Phase 5 (refactor to Dapr HTTP API); Phases 1-4 are migration steps

✅ **XV. Resource-aware local development**: Redpanda configured with 1 replica, no persistence, 512Mi memory limit for local

### Gates Status

**All gates passed** - Ready to proceed with Phase 0 research

## Project Structure

### Documentation (this feature)

```text
specs/010-advanced-task-features/
├── plan.md              # This file
├── research.md          # Phase 0 output (technology research)
├── data-model.md        # Phase 1 output (database schema)
├── quickstart.md        # Phase 1 output (setup instructions)
├── contracts/           # Phase 1 output (API contracts)
│   ├── tasks-api.md     # Extended task endpoints
│   └── events.md        # Dapr pub/sub event schemas
├── checklists/
│   └── requirements.md  # Specification quality checklist (complete)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   └── task.py              # Extended Task model (priority, tags, due_date, recurring)
│   ├── services/
│   │   ├── task_service.py      # Search/filter/sort logic
│   │   └── recurring_service.py # Recurring task automation
│   ├── api/
│   │   └── tasks.py             # Extended task endpoints
│   └── events/
│       └── reminder_publisher.py # Dapr pub/sub reminder events (Phase 5)
└── tests/
    ├── contract/
    ├── integration/
    └── unit/

frontend/
├── src/
│   ├── components/
│   │   ├── TaskForm.tsx         # Extended with priority, tags, due date, recurring
│   │   ├── TaskList.tsx         # Extended with filters, sort, search
│   │   ├── TaskFilters.tsx      # New: Filter UI component
│   │   └── TaskSearch.tsx       # New: Search UI component
│   ├── pages/
│   │   └── dashboard.tsx        # Updated with new components
│   └── services/
│       └── taskApi.ts           # Extended API client
└── tests/

helm/
├── values-dev.yaml              # Local development config (Redpanda 1 replica, no persistence)
├── values-prod.yaml             # Cloud production config (Redpanda HA, persistence enabled)
└── templates/
    ├── backend-deployment.yaml  # Dapr sidecar annotations (Phase 4+)
    ├── redpanda-statefulset.yaml # Messaging broker (Phase 3+)
    └── dapr-components/
        ├── kafka-pubsub.yaml    # Dapr pub/sub component (Phase 4+)
        └── postgres-state.yaml  # Dapr state component (Phase 4+)
```

**Structure Decision**: Web application structure (backend + frontend) selected. Backend handles business logic, API, and event publishing. Frontend provides UI for all advanced features. Helm charts manage deployment with environment-specific configurations.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations - all constitutional principles satisfied.

---

# Phase 0: Research

## Technology Research

### Database Schema Extensions

**Current Task Model** (from existing system):
```
Task:
  - id: UUID
  - user_id: UUID (FK to users)
  - title: String
  - description: String (nullable)
  - completed: Boolean
  - created_at: Timestamp
  - updated_at: Timestamp
```

**Required Extensions**:
```
Task (extended):
  + priority: Enum('high', 'medium', 'low') DEFAULT 'medium'
  + tags: Array<String> (PostgreSQL array type)
  + due_date: Timestamp (nullable)
  + recurring: Enum('none', 'daily', 'weekly', 'monthly') DEFAULT 'none'
  + parent_task_id: UUID (nullable, FK to tasks) # For tracking recurring task lineage
```

**Migration Strategy**: Alembic migration to add new columns with defaults; existing tasks default to medium priority, no tags, no due date, no recurrence.

### Search/Filter/Sort Implementation

**Search Approach**:
- PostgreSQL full-text search using `tsvector` for title and description
- Case-insensitive partial matching with `ILIKE` for simpler queries
- Index on `title` and `description` columns for performance

**Filter Approach**:
- SQL WHERE clauses for priority, completion status
- PostgreSQL array operators (`@>`, `&&`) for tag filtering
- Date comparison for due date filtering

**Sort Approach**:
- SQL ORDER BY clauses
- Custom sort logic for priority (high=1, medium=2, low=3)
- Overdue tasks prioritized in due date sort (CASE WHEN due_date < NOW())

### Recurring Task Automation

**Trigger Mechanism**:
- Backend service checks on task completion
- If `recurring != 'none'`, calculate next due date and create new task instance
- New task inherits: title, description, priority, tags, recurring pattern
- New task gets: new UUID, new created_at, completed=false, calculated due_date

**Date Calculation Logic**:
- Daily: `due_date + 1 day`
- Weekly: `due_date + 7 days`
- Monthly: `due_date + 1 month` (handle month-end edge cases per spec)

### Reminder Event Publishing

**Phase 1-2 Approach**: No reminders (deferred to Phase 3+)

**Phase 3-4 Approach**: Direct Kafka publishing (temporary)
- Topic: `tasks.reminder`
- Payload: `{task_id, user_id, title, due_date}`
- Trigger: Background job checks tasks with `due_date` approaching (e.g., 1 hour before)

**Phase 5+ Approach**: Dapr pub/sub HTTP API
- Endpoint: `POST http://localhost:3500/v1.0/publish/kafka-pubsub/tasks.reminder`
- Same payload structure
- Remove direct Kafka client dependencies

### Redpanda Configuration

**Local Development (values-dev.yaml)**:
```yaml
redpanda:
  replicas: 1
  persistence:
    enabled: false
  resources:
    limits:
      memory: 512Mi
      cpu: 500m
    requests:
      memory: 256Mi
      cpu: 250m
  config:
    auto_create_topics_enabled: true
```

**Cloud Production (values-prod.yaml)**:
```yaml
redpanda:
  replicas: 3
  persistence:
    enabled: true
    size: 10Gi
  resources:
    limits:
      memory: 2Gi
      cpu: 1000m
    requests:
      memory: 1Gi
      cpu: 500m
  config:
    auto_create_topics_enabled: true
```

### Dapr Configuration

**Installation** (Phase 4):
```bash
dapr init -k --enable-ha=false --enable-mtls=false
```

**Pub/Sub Component** (dapr-components/kafka-pubsub.yaml):
```yaml
apiVersion: dapr.io/v1alpha1
kind: Component
metadata:
  name: kafka-pubsub
spec:
  type: pubsub.kafka
  version: v1
  metadata:
  - name: brokers
    value: "redpanda:9092"
  - name: authType
    value: "none"
```

**State Component** (dapr-components/postgres-state.yaml):
```yaml
apiVersion: dapr.io/v1alpha1
kind: Component
metadata:
  name: postgres-state
spec:
  type: state.postgresql
  version: v1
  metadata:
  - name: connectionString
    secretKeyRef:
      name: postgres-secret
      key: connectionString
```

### Performance Considerations

**Database Indexes**:
- `CREATE INDEX idx_tasks_priority ON tasks(priority)`
- `CREATE INDEX idx_tasks_tags ON tasks USING GIN(tags)` (GIN index for array operations)
- `CREATE INDEX idx_tasks_due_date ON tasks(due_date) WHERE due_date IS NOT NULL`
- `CREATE INDEX idx_tasks_user_priority ON tasks(user_id, priority)` (composite for filtered queries)

**Query Optimization**:
- Limit result sets to 1000 tasks per query
- Use pagination for large task lists
- Cache tag lists in frontend to reduce API calls

**Event Publishing**:
- Batch reminder checks (every 5 minutes) to reduce database load
- Use database query to find tasks with `due_date BETWEEN NOW() AND NOW() + INTERVAL '1 hour'`

---

# Phase 1: Data Model & Contracts

## Data Model

See `data-model.md` (to be generated)

**Key Entities**:
- **Task** (extended): Adds priority, tags, due_date, recurring, parent_task_id
- **Priority** (enum): high, medium, low
- **RecurringPattern** (enum): none, daily, weekly, monthly
- **ReminderEvent** (event): Published to Dapr pub/sub when due date approaches

## API Contracts

See `contracts/tasks-api.md` (to be generated)

**Extended Endpoints**:
- `GET /api/{user_id}/tasks` - Add query params: `priority`, `tags`, `search`, `sort`, `filter`
- `POST /api/{user_id}/tasks` - Accept priority, tags, due_date, recurring in request body
- `PUT /api/{user_id}/tasks/{id}` - Accept priority, tags, due_date, recurring in request body
- `PATCH /api/{user_id}/tasks/{id}/complete` - Trigger recurring task creation if applicable

**New Endpoints**:
- `GET /api/{user_id}/tasks/tags` - List all unique tags used by user
- `GET /api/{user_id}/tasks/overdue` - List overdue tasks (due_date < NOW() AND completed = false)

See `contracts/events.md` (to be generated)

**Event Schemas**:
- `tasks.reminder` - Published when task due date approaches
- `tasks.recurring.created` - Published when recurring task instance is auto-created

## Quickstart

See `quickstart.md` (to be generated)

**Setup Steps**:
1. Run Alembic migration to add new Task columns
2. Update backend API with extended endpoints
3. Update frontend components with new UI elements
4. (Phase 3+) Deploy Redpanda with values-dev.yaml
5. (Phase 4+) Install Dapr and apply components
6. (Phase 5+) Refactor backend to use Dapr HTTP API

---

# Implementation Phases

## Phase 1: Model Update (No Messaging)

**Goal**: Add priority, tags, search, filter, sort capabilities without introducing messaging infrastructure.

**Backend Changes**:
1. Create Alembic migration to add columns: `priority`, `tags`, `due_date`, `recurring`, `parent_task_id`
2. Update `Task` SQLModel with new fields and defaults
3. Extend `task_service.py` with search/filter/sort logic:
   - `search_tasks(user_id, query)` - Full-text search on title/description
   - `filter_tasks(user_id, priority, tags, status)` - SQL WHERE clauses
   - `sort_tasks(tasks, sort_by)` - ORDER BY logic
4. Update API endpoints to accept new query parameters and request body fields
5. Add validation for priority enum, tags array, due_date format, recurring enum

**Frontend Changes**:
1. Update `TaskForm.tsx` to include:
   - Priority dropdown (high/medium/low)
   - Tags input (multi-select or comma-separated)
   - Due date picker (date + time)
   - Recurring dropdown (none/daily/weekly/monthly)
2. Create `TaskFilters.tsx` component:
   - Priority filter (checkboxes)
   - Tag filter (multi-select)
   - Status filter (done/not done)
3. Create `TaskSearch.tsx` component:
   - Search input with debounce
   - Real-time search results
4. Update `TaskList.tsx` to include:
   - Sort dropdown (due date/priority/created date/alphabetical)
   - Visual priority indicators (color coding or icons)
   - Overdue task highlighting (red text/warning icon)
5. Update `taskApi.ts` to send new query parameters and request body fields

**Database Migration**:
```sql
ALTER TABLE tasks ADD COLUMN priority VARCHAR(10) DEFAULT 'medium';
ALTER TABLE tasks ADD COLUMN tags TEXT[] DEFAULT '{}';
ALTER TABLE tasks ADD COLUMN due_date TIMESTAMP NULL;
ALTER TABLE tasks ADD COLUMN recurring VARCHAR(10) DEFAULT 'none';
ALTER TABLE tasks ADD COLUMN parent_task_id UUID NULL;
ALTER TABLE tasks ADD CONSTRAINT fk_parent_task FOREIGN KEY (parent_task_id) REFERENCES tasks(id);

CREATE INDEX idx_tasks_priority ON tasks(priority);
CREATE INDEX idx_tasks_tags ON tasks USING GIN(tags);
CREATE INDEX idx_tasks_due_date ON tasks(due_date) WHERE due_date IS NOT NULL;
CREATE INDEX idx_tasks_user_priority ON tasks(user_id, priority);
```

**Testing**:
- Unit tests for search/filter/sort logic
- Integration tests for extended API endpoints
- Frontend tests for new UI components

**Deliverable**: Users can assign priorities, add tags, search tasks, filter by priority/tags/status, and sort by various criteria. No recurring tasks or reminders yet.

---

## Phase 2: Recurring Logic (Backend Only)

**Goal**: Implement recurring task automation without messaging infrastructure.

**Backend Changes**:
1. Create `recurring_service.py` with:
   - `create_next_instance(task)` - Calculate next due date and create new task
   - `calculate_next_due_date(due_date, recurring_pattern)` - Date calculation logic
   - Handle month-end edge cases per spec (same day-of-month or last day if invalid)
2. Update `PATCH /api/{user_id}/tasks/{id}/complete` endpoint:
   - After marking task complete, check if `recurring != 'none'`
   - If recurring, call `create_next_instance(task)`
   - New task inherits: title, description, priority, tags, recurring pattern
   - New task gets: new UUID, completed=false, calculated due_date, parent_task_id=original_task_id
3. Add validation to prevent circular recurring references

**Testing**:
- Unit tests for date calculation logic (including month-end edge cases)
- Integration tests for recurring task creation on completion
- Test daily, weekly, monthly patterns

**Deliverable**: Users can create recurring tasks that automatically generate new instances when completed. Still no reminder events.

---

## Phase 3: Add Redpanda (Minimal Mode)

**Goal**: Introduce messaging infrastructure in minimal configuration for local development.

**Infrastructure Changes**:
1. Add Redpanda Helm chart to `helm/templates/redpanda-statefulset.yaml`
2. Configure `values-dev.yaml` with minimal settings:
   - replicas: 1
   - persistence: disabled
   - memory limit: 512Mi
   - cpu limit: 500m
3. Configure `values-prod.yaml` with production settings:
   - replicas: 3
   - persistence: enabled (10Gi)
   - memory limit: 2Gi
   - cpu limit: 1000m
4. Deploy Redpanda to local Minikube cluster
5. Verify Redpanda is running and accessible at `redpanda:9092`

**Backend Changes** (temporary, will be refactored in Phase 5):
1. Add `aiokafka` dependency to `requirements.txt`
2. Create `reminder_publisher.py` with direct Kafka client:
   - Connect to `redpanda:9092`
   - Publish to topic `tasks.reminder`
   - Payload: `{task_id, user_id, title, due_date}`
3. Create background job (e.g., Celery or APScheduler) to check for upcoming due dates:
   - Run every 5 minutes
   - Query: `SELECT * FROM tasks WHERE due_date BETWEEN NOW() AND NOW() + INTERVAL '1 hour' AND completed = false`
   - Publish reminder event for each task found

**Testing**:
- Verify Redpanda deployment in Minikube
- Test reminder event publishing to Kafka topic
- Verify events can be consumed (use `kafka-console-consumer` for testing)

**Deliverable**: Reminder events are published to Redpanda when tasks approach their due dates. Local deployment uses minimal resources (1 replica, no persistence).

---

## Phase 4: Install Dapr (Single Replica)

**Goal**: Add Dapr service mesh to enable abstracted communication patterns.

**Infrastructure Changes**:
1. Install Dapr to Kubernetes cluster:
   ```bash
   dapr init -k --enable-ha=false --enable-mtls=false
   ```
2. Create `helm/templates/dapr-components/kafka-pubsub.yaml`:
   - Type: `pubsub.kafka`
   - Brokers: `redpanda:9092`
   - Auth: none (local development)
3. Create `helm/templates/dapr-components/postgres-state.yaml`:
   - Type: `state.postgresql`
   - Connection string from Kubernetes Secret
4. Update backend deployment YAML to include Dapr sidecar annotations:
   ```yaml
   annotations:
     dapr.io/enabled: "true"
     dapr.io/app-id: "task-backend"
     dapr.io/app-port: "8000"
   ```
5. Verify Dapr sidecar is injected and running alongside backend pod

**Testing**:
- Verify Dapr control plane is running (`dapr status -k`)
- Verify Dapr sidecar is injected into backend pod
- Test Dapr pub/sub component with sample message
- Test Dapr state component with sample state operation

**Deliverable**: Dapr is installed and configured. Backend pod has Dapr sidecar. Pub/sub and state components are available but not yet used by application code.

---

## Phase 5: Refactor Backend (Dapr-First)

**Goal**: Replace direct Kafka client with Dapr HTTP API to achieve Dapr-first communication.

**Backend Changes**:
1. Remove `aiokafka` from `requirements.txt`
2. Remove direct Kafka client code from `reminder_publisher.py`
3. Refactor `reminder_publisher.py` to use Dapr HTTP API:
   ```python
   import httpx

   async def publish_reminder(task_id, user_id, title, due_date):
       async with httpx.AsyncClient() as client:
           response = await client.post(
               "http://localhost:3500/v1.0/publish/kafka-pubsub/tasks.reminder",
               json={
                   "task_id": str(task_id),
                   "user_id": str(user_id),
                   "title": title,
                   "due_date": due_date.isoformat()
               }
           )
           response.raise_for_status()
   ```
4. Update background job to use new Dapr-based publisher
5. Verify no direct Kafka imports remain in production code

**Testing**:
- Integration tests for Dapr pub/sub publishing
- Verify reminder events are still published to Redpanda (via Dapr)
- Verify no direct Kafka client dependencies in production code

**Deliverable**: Backend uses Dapr HTTP API for all pub/sub operations. No direct Kafka client in production branch. Achieves Principle XIV (Dapr-first communication).

---

## Phase 6: Cloud Deployment (Oracle OKE)

**Goal**: Deploy to Oracle Kubernetes Engine with full-scale production configuration.

**Infrastructure Changes**:
1. Create OKE cluster in Oracle Cloud
2. Configure `values-prod.yaml` for production:
   - Redpanda: 3 replicas, persistence enabled, 2Gi memory
   - Dapr: HA enabled, mTLS enabled
   - Backend: Multiple replicas, resource limits
   - Frontend: Multiple replicas, LoadBalancer service
3. Create Kubernetes Secrets for production:
   - Database connection string (Neon PostgreSQL)
   - JWT signing secret
   - Gemini API key
4. Deploy Helm chart with `values-prod.yaml`:
   ```bash
   helm upgrade --install task-app ./helm -f values-prod.yaml
   ```
5. Configure Ingress for external access
6. Set up monitoring and logging (Prometheus, Grafana, Loki)

**Testing**:
- Smoke tests in production environment
- Load testing with 1000+ concurrent users
- Verify HA failover (kill Redpanda pod, verify recovery)
- Verify data persistence (restart pods, verify data intact)

**Deliverable**: Application deployed to Oracle Cloud with full-scale production configuration. Local development remains lightweight (dev branch). Production runs with HA, persistence, and scalability (main branch).

---

# Migration Strategy

## Branch Strategy

- **dev branch**: Optimized for local development (values-dev.yaml)
  - Redpanda: 1 replica, no persistence, 512Mi memory
  - Dapr: Single replica, no HA
  - Total RAM usage: ≤2.5GB

- **main branch**: Optimized for cloud production (values-prod.yaml)
  - Redpanda: 3 replicas, persistence enabled, 2Gi memory
  - Dapr: HA enabled, mTLS enabled
  - Scalable resources

## Rollout Plan

1. **Phase 1-2**: Develop and test on dev branch (no messaging)
2. **Phase 3-4**: Add messaging infrastructure on dev branch (minimal config)
3. **Phase 5**: Refactor to Dapr-first on dev branch
4. **Phase 6**: Merge to main branch and deploy to cloud

## Rollback Plan

- Each phase is independently deployable
- If Phase 5 (Dapr refactor) fails, rollback to Phase 4 (direct Kafka)
- If Phase 3 (Redpanda) fails, rollback to Phase 2 (no messaging, no reminders)
- Database migrations are reversible via Alembic downgrade

---

# Risk Analysis

## Technical Risks

1. **Database Performance**: Search/filter/sort on large task lists may be slow
   - **Mitigation**: Add database indexes, implement pagination, limit result sets

2. **Recurring Task Edge Cases**: Month-end date calculations may have bugs
   - **Mitigation**: Comprehensive unit tests for all edge cases, manual testing

3. **Reminder Event Timing**: Background job may miss reminders if system is down
   - **Mitigation**: Use persistent job queue (Celery with Redis), implement retry logic

4. **Dapr Learning Curve**: Team may be unfamiliar with Dapr patterns
   - **Mitigation**: Phased approach allows learning incrementally, start with simple pub/sub

5. **Resource Constraints (Local)**: Redpanda + Dapr may exceed 2.5GB RAM limit
   - **Mitigation**: Minimal Redpanda config (512Mi), single Dapr replica, monitor resource usage

## Operational Risks

1. **Data Migration**: Adding new columns may lock table during migration
   - **Mitigation**: Run migration during low-traffic period, use `ADD COLUMN` with defaults (fast operation)

2. **Backward Compatibility**: Existing tasks need default values for new fields
   - **Mitigation**: Database defaults (priority='medium', tags='{}', recurring='none')

3. **Cloud Deployment Complexity**: OKE setup may have unexpected issues
   - **Mitigation**: Test in staging environment first, document all steps in quickstart.md

---

# Success Metrics

- **Phase 1**: Users can filter tasks by priority in <1 second (SC-003)
- **Phase 1**: Users can search 100+ tasks in <5 seconds (SC-002)
- **Phase 2**: Recurring tasks generate new instances within 1 minute (SC-007)
- **Phase 3**: Reminder events published with <1 minute variance (SC-008)
- **Phase 5**: No direct Kafka imports in production code (Principle XIV compliance)
- **Phase 6**: Application handles 1000+ concurrent users without degradation (SC-002)

---

# Next Steps

1. Run `/sp.tasks` to generate detailed task breakdown
2. Review and approve this plan
3. Begin Phase 1 implementation (model update)
4. Proceed through phases sequentially, validating each before moving to next
