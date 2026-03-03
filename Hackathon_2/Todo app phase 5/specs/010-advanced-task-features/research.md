# Research: Advanced Task Management Features

**Feature**: 010-advanced-task-features
**Date**: 2026-02-15
**Purpose**: Technology research and feasibility analysis for advanced task features

## Database Technology Research

### PostgreSQL Array Type for Tags

**Capability**: PostgreSQL supports native array types for storing multiple values in a single column.

**Syntax**:
```sql
CREATE TABLE tasks (
    tags TEXT[] DEFAULT '{}'
);
```

**Query Operations**:
- Contains any: `tags && ARRAY['work', 'urgent']` (overlaps operator)
- Contains all: `tags @> ARRAY['work']` (contains operator)
- Array length: `array_length(tags, 1)`

**Index Support**:
- GIN (Generalized Inverted Index): `CREATE INDEX idx_tags ON tasks USING GIN(tags)`
- Enables fast array containment queries
- Recommended for tag filtering

**Performance**: GIN indexes provide O(log n) lookup time for array containment queries.

**Decision**: Use PostgreSQL TEXT[] array type for tags with GIN index.

---

### Full-Text Search Options

**Option 1: PostgreSQL tsvector**
- Native full-text search capability
- Requires `tsvector` column and GIN index
- Supports stemming, ranking, phrase search
- More complex setup but better performance for large datasets

**Option 2: ILIKE Pattern Matching**
- Simple case-insensitive pattern matching
- Syntax: `WHERE title ILIKE '%search%'`
- Works with existing columns, no special setup
- Slower for large datasets but adequate for <1000 tasks per user

**Decision**: Start with ILIKE for simplicity (Phase 1). Migrate to tsvector if performance issues arise.

---

### Enum Types for Priority and Recurring

**PostgreSQL Enum**:
```sql
CREATE TYPE priority_level AS ENUM ('high', 'medium', 'low');
CREATE TYPE recurring_pattern AS ENUM ('none', 'daily', 'weekly', 'monthly');
```

**Pros**:
- Type safety at database level
- Efficient storage (4 bytes)
- Clear schema documentation

**Cons**:
- Harder to modify (requires ALTER TYPE)
- SQLModel/Pydantic enum mapping required

**Alternative: VARCHAR with CHECK constraint**:
```sql
priority VARCHAR(10) CHECK (priority IN ('high', 'medium', 'low'))
```

**Decision**: Use VARCHAR with CHECK constraint for flexibility. Enforce enum validation in Pydantic models.

---

## Recurring Task Date Calculation

### Monthly Edge Cases

**Problem**: Tasks scheduled for day 31 in months with fewer days.

**Test Cases**:
- Jan 31 → Feb 28 (non-leap year)
- Jan 31 → Feb 29 (leap year)
- Jan 31 → Mar 31 (next valid 31st)
- Jan 30 → Feb 28/29 (last day of Feb)
- Jan 15 → Feb 15 (no edge case)

**Algorithm** (per spec decision):
```python
def calculate_next_monthly_due_date(current_due_date):
    # Add 1 month
    next_month = current_due_date.month + 1
    next_year = current_due_date.year
    if next_month > 12:
        next_month = 1
        next_year += 1

    # Try to use same day-of-month
    target_day = current_due_date.day

    # Get last day of target month
    last_day = calendar.monthrange(next_year, next_month)[1]

    # Use target day or last day of month, whichever is smaller
    actual_day = min(target_day, last_day)

    return datetime(next_year, next_month, actual_day,
                   current_due_date.hour, current_due_date.minute)
```

**Testing Strategy**: Unit tests for all edge cases, including leap years.

---

## Messaging Infrastructure Research

### Redpanda vs Kafka

**Redpanda**:
- Kafka-compatible API (drop-in replacement)
- Written in C++ (lower memory footprint)
- No JVM required (faster startup)
- Better performance for small clusters
- Simpler configuration

**Kafka**:
- Industry standard
- Mature ecosystem
- Higher memory requirements (JVM)
- More complex configuration

**Decision**: Use Redpanda for Kafka-compatible messaging with lower resource footprint (aligns with Principle XV - resource-aware local development).

---

### Dapr Pub/Sub Abstraction

**Dapr Pub/Sub API**:
- HTTP endpoint: `POST http://localhost:3500/v1.0/publish/{pubsub-name}/{topic}`
- Abstracts underlying message broker (Kafka, Redis, RabbitMQ, etc.)
- Enables broker-agnostic application code
- Supports at-least-once delivery semantics

**Component Configuration**:
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
```

**Benefits**:
- Application code doesn't import Kafka libraries
- Can swap Redpanda for Redis Streams without code changes
- Aligns with Principle XIV (Dapr-first communication)

**Decision**: Use Dapr pub/sub abstraction for all event publishing (Phase 5+).

---

## Background Job Scheduling

### Options for Reminder Checks

**Option 1: APScheduler**
- Python library for scheduled jobs
- In-process scheduler (no external dependencies)
- Simple setup for periodic tasks
- Not distributed (single instance only)

**Option 2: Celery + Redis**
- Distributed task queue
- Supports multiple workers
- Requires Redis broker
- More complex setup

**Option 3: Kubernetes CronJob**
- Native Kubernetes resource
- Runs as separate pod
- Declarative configuration
- Good for periodic batch jobs

**Decision**: Start with APScheduler (Phase 3) for simplicity. Migrate to Kubernetes CronJob (Phase 6) for production scalability.

---

## Performance Optimization Research

### Database Indexing Strategy

**Required Indexes**:
1. `idx_tasks_priority` - B-tree index on priority column
2. `idx_tasks_tags` - GIN index on tags array
3. `idx_tasks_due_date` - Partial index on due_date (WHERE due_date IS NOT NULL)
4. `idx_tasks_user_priority` - Composite index on (user_id, priority)

**Query Patterns**:
- Filter by priority: Uses `idx_tasks_user_priority`
- Filter by tags: Uses `idx_tasks_tags`
- Sort by due date: Uses `idx_tasks_due_date`
- Search by title: Sequential scan (acceptable for <1000 tasks)

**Index Size Estimation**:
- 500 tasks per user × 10,000 users = 5M tasks
- B-tree index: ~100 bytes per entry = 500MB
- GIN index: ~200 bytes per entry = 1GB
- Total index size: ~1.5GB (acceptable)

---

## Frontend Component Research

### React Component Libraries for Filters

**Option 1: Headless UI + Tailwind**
- Unstyled, accessible components
- Full control over styling
- Lightweight
- Aligns with existing stack

**Option 2: Material-UI**
- Pre-styled components
- Comprehensive component library
- Heavier bundle size
- May conflict with existing styles

**Decision**: Use Headless UI + Tailwind for consistency with existing frontend architecture.

---

### Date Picker Libraries

**Option 1: react-datepicker**
- Lightweight (50KB)
- Customizable
- Supports date + time
- Good accessibility

**Option 2: date-fns + custom picker**
- More control
- Smaller bundle
- More development effort

**Decision**: Use react-datepicker for date/time selection (due dates).

---

## Resource Constraint Analysis

### Local Development Memory Budget

**Current Usage** (Phase 4 baseline):
- Minikube: 1GB
- Backend pod: 256MB
- Frontend pod: 256MB
- Neon connection: negligible (external)
- Total: ~1.5GB

**Phase 3 Addition (Redpanda)**:
- Redpanda pod: 512MB (configured limit)
- Total: ~2GB

**Phase 4 Addition (Dapr)**:
- Dapr control plane: 200MB
- Dapr sidecar (backend): 50MB
- Total: ~2.25GB

**Margin**: 250MB remaining (within 2.5GB limit)

**Conclusion**: Resource constraints satisfied with minimal Redpanda configuration.

---

## Security Considerations

### Tag Injection Prevention

**Risk**: Users could inject malicious tags (e.g., SQL injection via array literals).

**Mitigation**:
- Use parameterized queries (SQLModel handles this)
- Validate tag format: alphanumeric + hyphens only
- Limit tag length: 50 characters max
- Limit tags per task: 20 tags max

**Validation Regex**: `^[a-zA-Z0-9-]{1,50}$`

---

### Due Date Validation

**Risk**: Users could set due dates far in the past or future, causing issues.

**Mitigation**:
- Validate due date is not more than 1 year in the past
- Validate due date is not more than 10 years in the future
- Reject due dates with invalid timezone information

---

## Technology Stack Summary

| Component | Technology | Rationale |
|-----------|-----------|-----------|
| Database Arrays | PostgreSQL TEXT[] | Native support, GIN indexing |
| Search | ILIKE pattern matching | Simple, adequate for scale |
| Enums | VARCHAR + CHECK | Flexibility, Pydantic validation |
| Messaging | Redpanda | Kafka-compatible, low memory |
| Service Mesh | Dapr | Abstraction, Principle XIV |
| Background Jobs | APScheduler → K8s CronJob | Simple → scalable |
| Date Picker | react-datepicker | Lightweight, accessible |
| Filters UI | Headless UI + Tailwind | Consistent with stack |

---

## Feasibility Conclusion

All features are technically feasible with existing technology stack. No new external services required beyond Redpanda (Kafka-compatible) and Dapr (service mesh). Resource constraints satisfied with minimal configuration. Performance targets achievable with proper indexing and query optimization.

**Recommendation**: Proceed with implementation plan as designed.
