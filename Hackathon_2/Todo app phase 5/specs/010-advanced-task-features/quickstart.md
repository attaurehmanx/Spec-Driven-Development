# Quickstart Guide: Advanced Task Management Features

**Feature**: 010-advanced-task-features
**Date**: 2026-02-15
**Purpose**: Setup and deployment instructions for all implementation phases

## Prerequisites

- Docker Desktop installed and running
- Minikube installed (for local Kubernetes)
- kubectl CLI installed
- Helm 3.x installed
- Python 3.10+ (for backend development)
- Node.js 18+ (for frontend development)
- Git (for version control)

---

## Phase 1: Model Update (No Messaging)

### Backend Setup

**1. Create and apply database migration**:

```bash
cd backend

# Create migration
alembic revision -m "add_advanced_task_features"

# Edit the generated migration file with the schema from data-model.md

# Apply migration
alembic upgrade head
```

**2. Update SQLModel**:

```bash
# Edit backend/src/models/task.py
# Add new fields: priority, tags, due_date, recurring, parent_task_id
# See data-model.md for complete SQLModel definition
```

**3. Install dependencies**:

```bash
# No new dependencies for Phase 1
pip install -r requirements.txt
```

**4. Implement search/filter/sort logic**:

```bash
# Edit backend/src/services/task_service.py
# Add methods: search_tasks, filter_tasks, sort_tasks
```

**5. Update API endpoints**:

```bash
# Edit backend/src/api/tasks.py
# Add query parameters to GET /api/{user_id}/tasks
# Update request/response schemas
```

**6. Run backend**:

```bash
uvicorn src.main:app --reload --port 8000
```

### Frontend Setup

**1. Install dependencies**:

```bash
cd frontend

# Install date picker library
npm install react-datepicker
npm install @types/react-datepicker --save-dev

# Install Headless UI (if not already installed)
npm install @headlessui/react
```

**2. Update components**:

```bash
# Edit frontend/src/components/TaskForm.tsx
# Add: priority dropdown, tags input, due date picker, recurring dropdown

# Create frontend/src/components/TaskFilters.tsx
# Add: priority filter, tag filter, status filter

# Create frontend/src/components/TaskSearch.tsx
# Add: search input with debounce

# Edit frontend/src/components/TaskList.tsx
# Add: sort dropdown, priority indicators, overdue highlighting
```

**3. Update API client**:

```bash
# Edit frontend/src/services/taskApi.ts
# Add query parameters and new request body fields
```

**4. Run frontend**:

```bash
npm run dev
```

### Testing Phase 1

```bash
# Backend tests
cd backend
pytest tests/

# Frontend tests
cd frontend
npm test
```

**Validation**:
- ✅ Create task with priority, tags, due date
- ✅ Filter tasks by priority
- ✅ Filter tasks by tags
- ✅ Search tasks by title/description
- ✅ Sort tasks by priority, due date, created date, title
- ✅ Overdue tasks are highlighted

---

## Phase 2: Recurring Logic (Backend Only)

### Backend Setup

**1. Create recurring service**:

```bash
cd backend

# Create backend/src/services/recurring_service.py
# Implement: create_next_instance, calculate_next_due_date
```

**2. Update completion endpoint**:

```bash
# Edit backend/src/api/tasks.py
# Update PATCH /api/{user_id}/tasks/{id}/complete
# Add logic to create next recurring instance
```

### Testing Phase 2

```bash
cd backend

# Run unit tests for date calculation
pytest tests/unit/test_recurring_service.py

# Run integration tests for recurring task creation
pytest tests/integration/test_recurring_tasks.py
```

**Test Cases**:
- Daily recurring: Complete task, verify next instance created for tomorrow
- Weekly recurring: Complete task, verify next instance created for next week
- Monthly recurring: Complete task on Jan 31, verify Feb 28/29 instance
- Monthly recurring: Complete task on Jan 15, verify Feb 15 instance

**Validation**:
- ✅ Recurring tasks create new instances on completion
- ✅ Date calculation handles month-end edge cases correctly
- ✅ New instances inherit all properties except completion status

---

## Phase 3: Add Redpanda (Minimal Mode)

### Infrastructure Setup

**1. Start Minikube**:

```bash
minikube start --memory=3072 --cpus=2
```

**2. Add Redpanda Helm chart**:

```bash
cd helm

# Add Redpanda Helm repository
helm repo add redpanda https://charts.redpanda.com/
helm repo update

# Install Redpanda with minimal config
helm install redpanda redpanda/redpanda \
  --set replicas=1 \
  --set storage.persistentVolume.enabled=false \
  --set resources.memory.container.max=512Mi \
  --set resources.cpu.cores=0.5 \
  --namespace default
```

**3. Verify Redpanda is running**:

```bash
kubectl get pods -l app.kubernetes.io/name=redpanda

# Wait for pod to be Ready
kubectl wait --for=condition=ready pod -l app.kubernetes.io/name=redpanda --timeout=300s
```

**4. Test Redpanda connectivity**:

```bash
# Port forward to Redpanda
kubectl port-forward svc/redpanda 9092:9092

# In another terminal, test with kafka-console-producer
echo "test message" | kafka-console-producer --broker-list localhost:9092 --topic test-topic
```

### Backend Setup (Temporary Kafka Client)

**1. Install Kafka dependencies**:

```bash
cd backend

# Add to requirements.txt
echo "aiokafka==0.8.1" >> requirements.txt

pip install -r requirements.txt
```

**2. Create reminder publisher**:

```bash
# Create backend/src/events/reminder_publisher.py
# Implement direct Kafka publishing (temporary, will be refactored in Phase 5)
```

**3. Create background job**:

```bash
# Install APScheduler
echo "apscheduler==3.10.4" >> requirements.txt
pip install -r requirements.txt

# Create backend/src/jobs/reminder_job.py
# Implement periodic check for upcoming due dates
```

**4. Update main.py**:

```bash
# Edit backend/src/main.py
# Add APScheduler initialization
# Schedule reminder job to run every 5 minutes
```

### Testing Phase 3

```bash
# Verify Redpanda is accessible
kubectl exec -it redpanda-0 -- rpk topic list

# Create test topic
kubectl exec -it redpanda-0 -- rpk topic create tasks.reminder

# Run backend with reminder job
cd backend
uvicorn src.main:app --reload

# Create task with due date 1 hour in future
# Wait for reminder job to run (check logs)
# Verify event published to Kafka

# Consume events to verify
kubectl exec -it redpanda-0 -- rpk topic consume tasks.reminder
```

**Validation**:
- ✅ Redpanda pod running with 1 replica
- ✅ Memory usage ≤512Mi
- ✅ Reminder events published to tasks.reminder topic
- ✅ Background job runs every 5 minutes

---

## Phase 4: Install Dapr (Single Replica)

### Dapr Installation

**1. Install Dapr CLI**:

```bash
# macOS
brew install dapr/tap/dapr-cli

# Linux
wget -q https://raw.githubusercontent.com/dapr/cli/master/install/install.sh -O - | /bin/bash

# Windows
powershell -Command "iwr -useb https://raw.githubusercontent.com/dapr/cli/master/install/install.ps1 | iex"
```

**2. Initialize Dapr in Kubernetes**:

```bash
dapr init -k --enable-ha=false --enable-mtls=false --wait

# Verify Dapr installation
dapr status -k
```

**Expected output**:
```
NAME                   NAMESPACE    HEALTHY  STATUS   REPLICAS  VERSION  AGE  CREATED
dapr-sidecar-injector  dapr-system  True     Running  1         1.12.0   1m   2026-02-15 10:00:00
dapr-sentry            dapr-system  True     Running  1         1.12.0   1m   2026-02-15 10:00:00
dapr-operator          dapr-system  True     Running  1         1.12.0   1m   2026-02-15 10:00:00
dapr-placement         dapr-system  True     Running  1         1.12.0   1m   2026-02-15 10:00:00
```

### Dapr Component Configuration

**1. Create Kafka pub/sub component**:

```bash
cd helm/templates/dapr-components

# Create kafka-pubsub.yaml
cat <<EOF > kafka-pubsub.yaml
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
EOF

kubectl apply -f kafka-pubsub.yaml
```

**2. Create Postgres state component**:

```bash
# Create postgres-state.yaml
cat <<EOF > postgres-state.yaml
apiVersion: dapr.io/v1alpha1
kind: Component
metadata:
  name: postgres-state
  namespace: default
spec:
  type: state.postgresql
  version: v1
  metadata:
  - name: connectionString
    secretKeyRef:
      name: postgres-secret
      key: connectionString
EOF

kubectl apply -f postgres-state.yaml
```

**3. Update backend deployment**:

```bash
# Edit helm/templates/backend-deployment.yaml
# Add Dapr annotations:

apiVersion: apps/v1
kind: Deployment
metadata:
  name: task-backend
spec:
  template:
    metadata:
      annotations:
        dapr.io/enabled: "true"
        dapr.io/app-id: "task-backend"
        dapr.io/app-port: "8000"
    spec:
      containers:
      - name: backend
        image: task-backend:latest
        ports:
        - containerPort: 8000
```

**4. Deploy backend with Dapr sidecar**:

```bash
helm upgrade --install task-app ./helm -f values-dev.yaml
```

### Testing Phase 4

```bash
# Verify Dapr sidecar is injected
kubectl get pods
# Should see 2/2 containers (app + dapr sidecar)

# Check Dapr sidecar logs
kubectl logs <backend-pod-name> -c daprd

# Test Dapr pub/sub component
kubectl exec -it <backend-pod-name> -c backend -- curl -X POST \
  http://localhost:3500/v1.0/publish/kafka-pubsub/test-topic \
  -H "Content-Type: application/json" \
  -d '{"message": "test"}'

# Verify message in Redpanda
kubectl exec -it redpanda-0 -- rpk topic consume test-topic
```

**Validation**:
- ✅ Dapr control plane running in dapr-system namespace
- ✅ Backend pod has Dapr sidecar (2/2 containers)
- ✅ Dapr pub/sub component configured
- ✅ Test message published via Dapr HTTP API

---

## Phase 5: Refactor Backend (Dapr-First)

### Backend Refactoring

**1. Remove Kafka dependencies**:

```bash
cd backend

# Remove from requirements.txt
sed -i '/aiokafka/d' requirements.txt

pip install -r requirements.txt
```

**2. Refactor reminder publisher**:

```bash
# Edit backend/src/events/reminder_publisher.py
# Replace AIOKafkaProducer with Dapr HTTP API

# Before (Phase 3-4):
from aiokafka import AIOKafkaProducer

async def publish_reminder(task_id, user_id, title, due_date):
    producer = AIOKafkaProducer(bootstrap_servers='redpanda:9092')
    await producer.start()
    await producer.send('tasks.reminder', ...)
    await producer.stop()

# After (Phase 5):
import httpx

async def publish_reminder(task_id, user_id, title, due_date):
    async with httpx.AsyncClient() as client:
        await client.post(
            "http://localhost:3500/v1.0/publish/kafka-pubsub/tasks.reminder",
            json={...}
        )
```

**3. Verify no Kafka imports**:

```bash
# Search for Kafka imports in production code
grep -r "from aiokafka" backend/src/
grep -r "import aiokafka" backend/src/

# Should return no results
```

**4. Update tests**:

```bash
# Edit backend/tests/integration/test_events.py
# Mock Dapr HTTP endpoint instead of Kafka producer
```

### Testing Phase 5

```bash
cd backend

# Run all tests
pytest tests/

# Verify reminder events still published
# Create task with due date 1 hour in future
# Check Dapr sidecar logs for publish events
kubectl logs <backend-pod-name> -c daprd | grep publish

# Consume from Redpanda to verify events
kubectl exec -it redpanda-0 -- rpk topic consume tasks.reminder
```

**Validation**:
- ✅ No aiokafka imports in production code
- ✅ Reminder events published via Dapr HTTP API
- ✅ Events still arrive in Redpanda
- ✅ All tests pass
- ✅ Principle XIV (Dapr-first communication) achieved

---

## Phase 6: Cloud Deployment (Oracle OKE)

### Oracle Cloud Setup

**1. Create OKE cluster**:

```bash
# Using Oracle Cloud Console or CLI
oci ce cluster create \
  --compartment-id <compartment-ocid> \
  --name task-app-cluster \
  --kubernetes-version v1.28.0 \
  --node-shape VM.Standard.E4.Flex \
  --node-shape-config '{"ocpus": 2, "memoryInGBs": 16}' \
  --quantity-per-subnet 2
```

**2. Configure kubectl**:

```bash
# Download kubeconfig
oci ce cluster create-kubeconfig \
  --cluster-id <cluster-ocid> \
  --file ~/.kube/oke-config

# Set context
export KUBECONFIG=~/.kube/oke-config
kubectl config use-context <oke-context>
```

**3. Create Kubernetes Secrets**:

```bash
# Database connection string
kubectl create secret generic postgres-secret \
  --from-literal=connectionString="postgresql://user:pass@neon-host/dbname"

# JWT signing secret
kubectl create secret generic jwt-secret \
  --from-literal=secret="your-jwt-secret-key"

# Gemini API key
kubectl create secret generic gemini-secret \
  --from-literal=apiKey="your-gemini-api-key"
```

### Production Deployment

**1. Update Helm values for production**:

```bash
cd helm

# Edit values-prod.yaml
# Set production configurations:
# - Redpanda: 3 replicas, persistence enabled
# - Dapr: HA enabled, mTLS enabled
# - Backend: Multiple replicas, resource limits
# - Frontend: Multiple replicas, LoadBalancer service
```

**2. Install Dapr with HA**:

```bash
dapr init -k --enable-ha=true --enable-mtls=true --wait
```

**3. Deploy application**:

```bash
helm upgrade --install task-app ./helm \
  -f values-prod.yaml \
  --namespace default \
  --create-namespace
```

**4. Configure Ingress**:

```bash
# Install NGINX Ingress Controller
helm repo add ingress-nginx https://kubernetes.github.io/ingress-nginx
helm install ingress-nginx ingress-nginx/ingress-nginx

# Create Ingress resource
kubectl apply -f helm/templates/ingress.yaml
```

**5. Verify deployment**:

```bash
# Check all pods are running
kubectl get pods

# Check services
kubectl get svc

# Get external IP
kubectl get ingress
```

### Testing Production

```bash
# Smoke tests
curl https://api.taskapp.example.com/health

# Create test task
curl -X POST https://api.taskapp.example.com/api/{user_id}/tasks \
  -H "Authorization: Bearer <token>" \
  -H "Content-Type: application/json" \
  -d '{"title": "Test task", "priority": "high"}'

# Load testing (optional)
k6 run load-test.js
```

**Validation**:
- ✅ All pods running with multiple replicas
- ✅ Redpanda has 3 replicas with persistence
- ✅ Dapr HA enabled
- ✅ Application accessible via Ingress
- ✅ Load test passes (1000+ concurrent users)

---

## Monitoring and Observability

### Prometheus and Grafana

**1. Install Prometheus**:

```bash
helm repo add prometheus-community https://prometheus-community.github.io/helm-charts
helm install prometheus prometheus-community/kube-prometheus-stack
```

**2. Access Grafana**:

```bash
kubectl port-forward svc/prometheus-grafana 3000:80

# Default credentials: admin / prom-operator
```

**3. Import Dapr dashboards**:

- Go to Grafana → Dashboards → Import
- Import dashboard ID: 19862 (Dapr System Dashboard)
- Import dashboard ID: 19863 (Dapr Sidecar Dashboard)

### Logging with Loki

**1. Install Loki**:

```bash
helm repo add grafana https://grafana.github.io/helm-charts
helm install loki grafana/loki-stack
```

**2. View logs in Grafana**:

- Add Loki data source
- Query: `{app="task-backend"}`

---

## Troubleshooting

### Common Issues

**Issue**: Redpanda pod stuck in Pending
```bash
# Check events
kubectl describe pod redpanda-0

# Common cause: Insufficient resources
# Solution: Increase Minikube memory
minikube delete
minikube start --memory=4096
```

**Issue**: Dapr sidecar not injected
```bash
# Check annotations
kubectl get deployment task-backend -o yaml | grep dapr

# Solution: Add annotations to deployment
kubectl patch deployment task-backend -p '{"spec":{"template":{"metadata":{"annotations":{"dapr.io/enabled":"true"}}}}}'
```

**Issue**: Reminder events not published
```bash
# Check background job logs
kubectl logs <backend-pod-name> -c backend | grep reminder

# Check Dapr sidecar logs
kubectl logs <backend-pod-name> -c daprd

# Verify Dapr component
kubectl get component kafka-pubsub -o yaml
```

**Issue**: Database migration fails
```bash
# Check connection
kubectl exec -it <backend-pod-name> -- python -c "import asyncpg; print('OK')"

# Run migration manually
kubectl exec -it <backend-pod-name> -- alembic upgrade head
```

---

## Resource Monitoring

### Check Resource Usage

```bash
# Overall cluster resources
kubectl top nodes

# Pod resources
kubectl top pods

# Redpanda memory usage
kubectl top pod -l app.kubernetes.io/name=redpanda
```

**Expected Local Development Usage**:
- Minikube: ~1GB
- Backend: ~256MB
- Frontend: ~256MB
- Redpanda: ~512MB
- Dapr control plane: ~200MB
- Dapr sidecars: ~50MB each
- **Total**: ~2.3GB (within 2.5GB limit)

---

## Rollback Procedures

### Rollback Helm Release

```bash
# List releases
helm list

# Rollback to previous version
helm rollback task-app

# Rollback to specific revision
helm rollback task-app 3
```

### Rollback Database Migration

```bash
# Downgrade one revision
kubectl exec -it <backend-pod-name> -- alembic downgrade -1

# Downgrade to specific revision
kubectl exec -it <backend-pod-name> -- alembic downgrade 009_containerization
```

---

## Next Steps

After completing all phases:

1. Run `/sp.tasks` to generate detailed task breakdown
2. Begin implementation starting with Phase 1
3. Validate each phase before proceeding to next
4. Monitor resource usage throughout development
5. Document any deviations from plan in `specs/010-advanced-task-features/notes.md`
