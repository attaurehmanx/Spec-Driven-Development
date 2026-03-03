# Event Contracts: Dapr Pub/Sub Events

**Feature**: 010-advanced-task-features
**Date**: 2026-02-15
**Purpose**: Event schema specifications for Dapr pub/sub messaging

## Overview

This document defines the event schemas for asynchronous communication via Dapr pub/sub. Events are published to Redpanda (Kafka-compatible) through the Dapr HTTP API.

**Dapr Component**: `kafka-pubsub`
**Broker**: Redpanda at `redpanda:9092`

---

## Publishing Events

### Dapr HTTP API

**Endpoint**: `POST http://localhost:3500/v1.0/publish/{pubsub-name}/{topic}`

**Example**:
```bash
curl -X POST http://localhost:3500/v1.0/publish/kafka-pubsub/tasks.reminder \
  -H "Content-Type: application/json" \
  -d '{
    "task_id": "123e4567-e89b-12d3-a456-426614174000",
    "user_id": "550e8400-e29b-41d4-a716-446655440000",
    "title": "Complete project proposal",
    "due_date": "2026-02-20T17:00:00Z"
  }'
```

### Python Example

```python
import httpx
from datetime import datetime
from uuid import UUID

async def publish_event(pubsub_name: str, topic: str, data: dict):
    """Publish event to Dapr pub/sub"""
    dapr_url = f"http://localhost:3500/v1.0/publish/{pubsub_name}/{topic}"

    async with httpx.AsyncClient() as client:
        response = await client.post(dapr_url, json=data)
        response.raise_for_status()
```

---

## Event Schemas

### 1. Task Reminder Event

**Topic**: `tasks.reminder`

**Description**: Published when a task's due date is approaching (typically 1 hour before due date).

**Schema**:

```json
{
  "event_type": "task.reminder",
  "event_id": "uuid",
  "timestamp": "ISO 8601 datetime",
  "data": {
    "task_id": "uuid",
    "user_id": "uuid",
    "title": "string",
    "description": "string (nullable)",
    "due_date": "ISO 8601 datetime",
    "priority": "high | medium | low",
    "tags": ["string"]
  }
}
```

**Example**:

```json
{
  "event_type": "task.reminder",
  "event_id": "789e4567-e89b-12d3-a456-426614174888",
  "timestamp": "2026-02-20T16:00:00Z",
  "data": {
    "task_id": "123e4567-e89b-12d3-a456-426614174000",
    "user_id": "550e8400-e29b-41d4-a716-446655440000",
    "title": "Complete project proposal",
    "description": "Finalize Q1 project proposal for client review",
    "due_date": "2026-02-20T17:00:00Z",
    "priority": "high",
    "tags": ["work", "urgent"]
  }
}
```

**Publishing Trigger**: Background job checks for tasks with `due_date BETWEEN NOW() AND NOW() + INTERVAL '1 hour'` every 5 minutes.

**Consumers**: Notification services (email, SMS, push notifications, in-app notifications)

**Idempotency**: Consumers should deduplicate based on `event_id` to handle at-least-once delivery.

---

### 2. Recurring Task Created Event

**Topic**: `tasks.recurring.created`

**Description**: Published when a new recurring task instance is automatically created after the previous instance is completed.

**Schema**:

```json
{
  "event_type": "task.recurring.created",
  "event_id": "uuid",
  "timestamp": "ISO 8601 datetime",
  "data": {
    "task_id": "uuid",
    "user_id": "uuid",
    "parent_task_id": "uuid",
    "title": "string",
    "description": "string (nullable)",
    "due_date": "ISO 8601 datetime",
    "priority": "high | medium | low",
    "tags": ["string"],
    "recurring": "daily | weekly | monthly"
  }
}
```

**Example**:

```json
{
  "event_type": "task.recurring.created",
  "event_id": "999e4567-e89b-12d3-a456-426614174777",
  "timestamp": "2026-02-15T14:05:00Z",
  "data": {
    "task_id": "789e4567-e89b-12d3-a456-426614174999",
    "user_id": "550e8400-e29b-41d4-a716-446655440000",
    "parent_task_id": "123e4567-e89b-12d3-a456-426614174000",
    "title": "Weekly team meeting",
    "description": "Discuss project progress",
    "due_date": "2026-02-22T14:00:00Z",
    "priority": "medium",
    "tags": ["work", "meeting"],
    "recurring": "weekly"
  }
}
```

**Publishing Trigger**: When a recurring task is marked complete via `PATCH /api/{user_id}/tasks/{task_id}/complete`.

**Consumers**: Analytics services, audit logs, notification services (to inform user of new task creation)

**Idempotency**: Consumers should deduplicate based on `event_id`.

---

### 3. Task Priority Changed Event (Optional - Future)

**Topic**: `tasks.priority.changed`

**Description**: Published when a task's priority is changed from one level to another.

**Schema**:

```json
{
  "event_type": "task.priority.changed",
  "event_id": "uuid",
  "timestamp": "ISO 8601 datetime",
  "data": {
    "task_id": "uuid",
    "user_id": "uuid",
    "title": "string",
    "old_priority": "high | medium | low",
    "new_priority": "high | medium | low"
  }
}
```

**Note**: Not implemented in Phase 1-6. Reserved for future analytics use cases.

---

### 4. Task Overdue Event (Optional - Future)

**Topic**: `tasks.overdue`

**Description**: Published when a task becomes overdue (due_date < NOW() AND completed = false).

**Schema**:

```json
{
  "event_type": "task.overdue",
  "event_id": "uuid",
  "timestamp": "ISO 8601 datetime",
  "data": {
    "task_id": "uuid",
    "user_id": "uuid",
    "title": "string",
    "due_date": "ISO 8601 datetime",
    "days_overdue": "integer"
  }
}
```

**Note**: Not implemented in Phase 1-6. Reserved for future notification use cases.

---

## Event Metadata

All events include standard metadata fields:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| event_type | string | Yes | Event type identifier (e.g., "task.reminder") |
| event_id | UUID | Yes | Unique event identifier for idempotency |
| timestamp | datetime | Yes | ISO 8601 timestamp when event was published |
| data | object | Yes | Event-specific payload |

---

## Dapr Pub/Sub Component Configuration

**File**: `helm/templates/dapr-components/kafka-pubsub.yaml`

```yaml
apiVersion: dapr.io/v1alpha1
kind: Component
metadata:
  name: kafka-pubsub
  namespace: default
spec:
  type: pubsub.kafka
  version: v1
  metadata:
  - name: brokers
    value: "redpanda:9092"
  - name: authType
    value: "none"
  - name: consumerGroup
    value: "task-service"
  - name: clientId
    value: "task-backend"
  - name: maxMessageBytes
    value: "1048576"  # 1MB
```

---

## Topic Configuration

### Redpanda Topic Settings

**Local Development** (values-dev.yaml):
```yaml
redpanda:
  topics:
    - name: tasks.reminder
      partitions: 1
      replicationFactor: 1
      retentionMs: 86400000  # 24 hours
    - name: tasks.recurring.created
      partitions: 1
      replicationFactor: 1
      retentionMs: 604800000  # 7 days
```

**Cloud Production** (values-prod.yaml):
```yaml
redpanda:
  topics:
    - name: tasks.reminder
      partitions: 3
      replicationFactor: 3
      retentionMs: 604800000  # 7 days
    - name: tasks.recurring.created
      partitions: 3
      replicationFactor: 3
      retentionMs: 2592000000  # 30 days
```

---

## Consumer Implementation

### Subscribing to Events

**Dapr Subscription Configuration**:

```yaml
apiVersion: dapr.io/v1alpha1
kind: Subscription
metadata:
  name: task-reminder-subscription
spec:
  pubsubname: kafka-pubsub
  topic: tasks.reminder
  route: /events/reminder
  metadata:
    rawPayload: "false"
```

**FastAPI Endpoint**:

```python
from fastapi import APIRouter, Request

router = APIRouter()

@router.post("/events/reminder")
async def handle_reminder_event(request: Request):
    """Handle task reminder events from Dapr pub/sub"""
    event = await request.json()

    # Extract event data
    event_type = event.get("type")  # Dapr wraps event_type as "type"
    data = event.get("data", {})

    task_id = data.get("task_id")
    user_id = data.get("user_id")
    title = data.get("title")
    due_date = data.get("due_date")

    # Process reminder (e.g., send notification)
    await send_notification(user_id, f"Reminder: {title} is due at {due_date}")

    # Return 200 to acknowledge receipt
    return {"status": "success"}
```

---

## Error Handling

### Retry Policy

Dapr provides automatic retry with exponential backoff:

- Initial retry delay: 1 second
- Max retry delay: 30 seconds
- Max retry attempts: 10

**Configuration**:

```yaml
apiVersion: dapr.io/v1alpha1
kind: Subscription
metadata:
  name: task-reminder-subscription
spec:
  pubsubname: kafka-pubsub
  topic: tasks.reminder
  route: /events/reminder
  deadLetterTopic: tasks.reminder.dlq
  metadata:
    maxRetries: "10"
    retryBackoff: "exponential"
```

### Dead Letter Queue

Failed events (after max retries) are sent to dead letter topic:

**Topic**: `tasks.reminder.dlq`

Consumers should monitor DLQ for failed events and investigate root causes.

---

## Monitoring and Observability

### Metrics to Track

1. **Event Publishing**:
   - Events published per topic
   - Publishing latency (p50, p95, p99)
   - Publishing errors

2. **Event Consumption**:
   - Events consumed per topic
   - Processing latency
   - Processing errors
   - Retry count

3. **Dead Letter Queue**:
   - Events in DLQ
   - DLQ growth rate

### Logging

All event publishing and consumption should log:

```json
{
  "timestamp": "2026-02-15T14:05:00Z",
  "level": "INFO",
  "event_type": "task.reminder",
  "event_id": "789e4567-e89b-12d3-a456-426614174888",
  "topic": "tasks.reminder",
  "action": "published",
  "task_id": "123e4567-e89b-12d3-a456-426614174000",
  "user_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

---

## Security Considerations

### Event Data Sensitivity

- Events contain user data (task titles, descriptions)
- Do NOT include sensitive data (passwords, tokens) in events
- Consider encrypting event payloads for compliance requirements

### Access Control

- Dapr pub/sub components should be scoped to specific namespaces
- Use Kubernetes RBAC to restrict access to Dapr components
- Enable mTLS for Dapr sidecar communication in production

---

## Testing

### Unit Testing

Mock Dapr HTTP API for unit tests:

```python
import pytest
from unittest.mock import AsyncMock, patch

@pytest.mark.asyncio
async def test_publish_reminder_event():
    with patch('httpx.AsyncClient.post') as mock_post:
        mock_post.return_value.status_code = 200

        await publish_reminder_event(
            task_id="123e4567-e89b-12d3-a456-426614174000",
            user_id="550e8400-e29b-41d4-a716-446655440000",
            title="Test task",
            due_date="2026-02-20T17:00:00Z"
        )

        mock_post.assert_called_once()
```

### Integration Testing

Use Dapr test containers:

```python
import pytest
from testcontainers.compose import DockerCompose

@pytest.fixture(scope="session")
def dapr_environment():
    with DockerCompose(".", compose_file_name="docker-compose.test.yml") as compose:
        compose.wait_for("http://localhost:3500/v1.0/healthz")
        yield compose
```

---

## Migration Notes

### Phase 3-4: Direct Kafka Publishing (Temporary)

During Phase 3-4, events are published directly to Kafka using `aiokafka`:

```python
from aiokafka import AIOKafkaProducer
import json

producer = AIOKafkaProducer(bootstrap_servers='redpanda:9092')
await producer.start()

await producer.send(
    'tasks.reminder',
    json.dumps(event_data).encode('utf-8')
)

await producer.stop()
```

### Phase 5: Dapr Refactor

Replace direct Kafka code with Dapr HTTP API (as shown in Publishing Events section above).

**Migration Checklist**:
- [ ] Remove `aiokafka` from requirements.txt
- [ ] Replace all `AIOKafkaProducer` usage with Dapr HTTP API
- [ ] Update tests to mock Dapr HTTP endpoint
- [ ] Verify no direct Kafka imports remain in production code

---

## Event Schema Versioning

### Version Strategy

Events include implicit version in `event_type` field:

- `task.reminder` (v1 - current)
- `task.reminder.v2` (future version)

### Backward Compatibility

When adding new fields:
- Add as optional fields
- Consumers should ignore unknown fields
- Maintain old event types for 6 months before deprecation

### Breaking Changes

When making breaking changes:
- Create new event type with version suffix
- Publish both old and new event types during transition period
- Deprecate old event type after all consumers migrate
