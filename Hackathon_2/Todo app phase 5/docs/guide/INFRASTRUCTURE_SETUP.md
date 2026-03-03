# Phase 6: Due Dates and Reminders - Infrastructure Setup Guide

This guide walks through setting up the infrastructure required for Phase 6: Due Dates and Reminders.

## Prerequisites

- Docker Desktop installed and running
- kubectl CLI installed
- Helm 3.x installed
- Minikube installed
- Dapr CLI installed

## Step-by-Step Setup

### 1. Start Minikube Cluster

```bash
# Start Minikube with sufficient resources
minikube start --memory=1900mb --cpus=2 --disk-size=16g --driver=docker

# Verify cluster is running
kubectl cluster-info
kubectl get nodes
```

### 2. Install Redpanda (Kafka-compatible Message Broker)

```bash
# Add Redpanda Helm repository
helm repo add redpanda https://charts.redpanda.com
helm repo update

# Install Redpanda with minimal configuration
helm install redpanda redpanda/redpanda -f helm/values-dev.yaml

# Wait for Redpanda pod to be ready (may take 2-3 minutes)
kubectl wait --for=condition=ready pod -l app.kubernetes.io/name=redpanda --timeout=300s

# Verify Redpanda is running
kubectl get pods -l app.kubernetes.io/name=redpanda
```

### 3. Create Kafka Topic

```bash
# Create tasks.reminder topic
kubectl exec redpanda-0 -- rpk topic create tasks.reminder --partitions 3 --replicas 1

# Verify topic was created
kubectl exec redpanda-0 -- rpk topic list
```

### 4. Install Dapr to Kubernetes

```bash
# Initialize Dapr on Kubernetes (without HA and mTLS for development)
dapr init -k --enable-ha=false --enable-mtls=false

# Wait for Dapr control plane to be ready
kubectl wait --for=condition=ready pod -l app=dapr-operator -n dapr-system --timeout=300s

# Verify Dapr is running
dapr status -k
```

Expected output:
```
NAME                   NAMESPACE    HEALTHY  STATUS   REPLICAS  VERSION  AGE  CREATED
dapr-operator          dapr-system  True     Running  1         1.x.x    Xs   xxxx-xx-xx xx:xx.xx
dapr-sidecar-injector  dapr-system  True     Running  1         1.x.x    Xs   xxxx-xx-xx xx:xx.xx
dapr-sentry            dapr-system  True     Running  1         1.x.x    Xs   xxxx-xx-xx xx:xx.xx
dapr-placement-server  dapr-system  True     Running  1         1.x.x    Xs   xxxx-xx-xx xx:xx.xx
```

### 5. Deploy Dapr Components

```bash
# Apply Kafka pub/sub component
kubectl apply -f helm/templates/dapr-components/kafka-pubsub.yaml

# Apply PostgreSQL state component
kubectl apply -f helm/templates/dapr-components/postgres-state.yaml

# Verify components are created
kubectl get components
```

### 6. Build and Deploy Backend with Dapr Sidecar

```bash
# Build backend Docker image
cd backend
docker build -t task-management-backend:latest .

# Load image into Minikube
minikube image load task-management-backend:latest

# Create secrets for API keys
kubectl create secret generic api-secrets \
  --from-literal=gemini-api-key=YOUR_GEMINI_API_KEY \
  --from-literal=auth-secret=YOUR_AUTH_SECRET

# Deploy backend with Dapr sidecar
kubectl apply -f helm/templates/backend-deployment.yaml

# Wait for deployment to be ready
kubectl wait --for=condition=available deployment/task-management-backend --timeout=300s

# Verify Dapr sidecar is injected (should show 2/2 containers)
kubectl get pods -l app=task-management
```

Expected output:
```
NAME                                        READY   STATUS    RESTARTS   AGE
task-management-backend-xxxxxxxxxx-xxxxx    2/2     Running   0          30s
```

### 7. Verify Reminder System

```bash
# Check backend logs
kubectl logs -l app=task-management -c backend --tail=50

# Check Dapr sidecar logs
kubectl logs -l app=task-management -c daprd --tail=50

# Check APScheduler is running
kubectl logs -l app=task-management -c backend | grep "APScheduler started"

# Check reminder job execution
kubectl logs -l app=task-management -c backend | grep "reminder"
```

### 8. Test Reminder Publishing

```bash
# Port-forward to access backend API
kubectl port-forward svc/task-management-backend 8001:8001

# In another terminal, create a task with due date (1 hour from now)
curl -X POST http://localhost:8001/api/tasks \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -d '{
    "title": "Test Reminder Task",
    "description": "This task should trigger a reminder",
    "priority": "high",
    "due_date": "2024-01-01T15:00:00Z",
    "completed": false
  }'

# Wait 5 minutes for reminder job to run
# Check logs for reminder event
kubectl logs -l app=task-management -c backend | grep "Published reminder"
```

### 9. Monitor Kafka Topics

```bash
# Consume messages from tasks.reminder topic
kubectl exec -it redpanda-0 -- rpk topic consume tasks.reminder

# You should see reminder events like:
# {
#   "task_id": 123,
#   "user_id": "user-uuid",
#   "title": "Test Reminder Task",
#   "due_date": "2024-01-01T15:00:00Z",
#   "priority": "high",
#   "event_type": "task_reminder",
#   "timestamp": "2024-01-01T14:05:00Z"
# }
```

## Troubleshooting

### Redpanda Pod Not Starting

```bash
# Check pod status
kubectl describe pod redpanda-0

# Check logs
kubectl logs redpanda-0

# Common issue: Insufficient memory
# Solution: Increase Minikube memory or reduce Redpanda memory limits
```

### Dapr Sidecar Not Injected

```bash
# Verify Dapr annotations in deployment
kubectl get deployment task-management-backend -o yaml | grep dapr

# Check Dapr sidecar injector logs
kubectl logs -n dapr-system -l app=dapr-sidecar-injector

# Restart deployment
kubectl rollout restart deployment/task-management-backend
```

### Reminder Job Not Running

```bash
# Check APScheduler logs
kubectl logs -l app=task-management -c backend | grep "APScheduler"

# Verify database connection
kubectl logs -l app=task-management -c backend | grep "Database"

# Check for errors in reminder job
kubectl logs -l app=task-management -c backend | grep "reminder_job"
```

### Kafka Connection Issues

```bash
# Verify Redpanda service is accessible
kubectl get svc redpanda

# Test connection from backend pod
kubectl exec -it deployment/task-management-backend -c backend -- \
  curl redpanda-0.redpanda.default.svc.cluster.local:9092

# Check Kafka pub/sub component
kubectl describe component kafka-pubsub
```

## Cleanup

```bash
# Delete backend deployment
kubectl delete -f helm/templates/backend-deployment.yaml

# Delete Dapr components
kubectl delete -f helm/templates/dapr-components/

# Uninstall Dapr
dapr uninstall -k

# Uninstall Redpanda
helm uninstall redpanda

# Stop Minikube
minikube stop

# Delete Minikube cluster (optional)
minikube delete
```

## Environment Variables

The backend requires the following environment variables:

- `DATABASE_URL`: PostgreSQL connection string
- `KAFKA_BOOTSTRAP_SERVERS`: Kafka/Redpanda bootstrap servers
- `GEMINI_API_KEY`: Google Gemini API key for AI features
- `BETTER_AUTH_SECRET`: Secret key for authentication

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     Kubernetes Cluster                       │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Backend Pod (2/2 containers)                        │  │
│  │  ┌────────────────┐  ┌──────────────────────────┐   │  │
│  │  │  FastAPI App   │  │  Dapr Sidecar            │   │  │
│  │  │  - APScheduler │◄─┤  - Kafka Pub/Sub         │   │  │
│  │  │  - Reminder Job│  │  - State Management      │   │  │
│  │  └────────┬───────┘  └──────────┬───────────────┘   │  │
│  └───────────┼──────────────────────┼───────────────────┘  │
│              │                      │                       │
│              │                      │                       │
│  ┌───────────▼──────────┐  ┌───────▼────────────────────┐ │
│  │  PostgreSQL          │  │  Redpanda (Kafka)          │ │
│  │  - Task Data         │  │  - tasks.reminder topic    │ │
│  │  - User Data         │  │  - Event Streaming         │ │
│  └──────────────────────┘  └────────────────────────────┘ │
│                                                              │
└─────────────────────────────────────────────────────────────┘

Flow:
1. APScheduler runs every 5 minutes
2. Reminder job queries PostgreSQL for tasks due within 1 hour
3. For each task, publishes reminder event to Kafka via Dapr
4. Events are consumed by notification service (future implementation)
```

## Next Steps

After infrastructure is set up:

1. Test creating tasks with due dates via frontend
2. Verify overdue highlighting works correctly
3. Monitor reminder events in Kafka
4. Implement notification consumer (future phase)
5. Add email/SMS notification delivery (future phase)

## Phase 5: Dapr Refactor (Future)

After Phase 6 is working, refactor to use Dapr HTTP API instead of direct Kafka:

1. Remove aiokafka dependency
2. Update reminder_publisher.py to use Dapr HTTP API:
   ```python
   # POST http://localhost:3500/v1.0/publish/kafka-pubsub/tasks.reminder
   ```
3. Verify no direct Kafka imports remain

This makes the system more portable and cloud-native.
