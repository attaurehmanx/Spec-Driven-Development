---

description: "Task list for containerization strategy implementation"
---

# Tasks: Containerization Strategy

**Input**: Design documents from `/specs/009-containerization/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, quickstart.md

**Tests**: Tests are NOT requested in the specification. This is an infrastructure feature focused on containerization.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/`, `frontend-app/` at repository root
- Paths shown below use actual project structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 [P] Verify backend/.dockerignore exists and contains required exclusions (venv, __pycache__, .env, .git, tests)
- [ ] T002 Create frontend-app/.dockerignore with exclusions (node_modules, .next, .git, .env*.local, .vercel, .vscode, .idea, coverage, .DS_Store)

---

## Phase 2: User Story 1 - Backend Service Containerization (Priority: P1) 🎯 MVP

**Goal**: Package FastAPI backend into optimized container image with Python 3.11-slim runtime

**Independent Test**: Build backend container image, run with environment variables (DATABASE_URL, GEMINI_API_KEY, BETTER_AUTH_SECRET), verify API responds on port 8000 and health check passes

### Implementation for User Story 1

- [ ] T003 [US1] Read current backend/Dockerfile to understand existing implementation
- [ ] T004 [US1] Update backend/Dockerfile EXPOSE directive from port 7860 to port 8000 per FR-003
- [ ] T005 [US1] Update backend/Dockerfile CMD to use "main:app" instead of "app:app" (correct entry point)
- [ ] T006 [US1] Remove stateful directory creation (uploads/avatars) from backend/Dockerfile RUN command to comply with Principle X (Container immutability)
- [ ] T007 [US1] Update backend/Dockerfile HEALTHCHECK to check port 8000 instead of 7860
- [ ] T008 [US1] Add comment block to backend/Dockerfile documenting required environment variables (DATABASE_URL, GEMINI_API_KEY, BETTER_AUTH_SECRET, BETTER_AUTH_URL)
- [ ] T009 [US1] Build backend container image using "docker build -t todo-backend:latest ./backend"
- [ ] T010 [US1] Verify backend image size is under 500MB using "docker images" command (SC-003)
- [ ] T011 [US1] Run backend container locally with test environment variables and verify startup time under 10 seconds (SC-001)
- [ ] T012 [US1] Test backend container health check endpoint responds correctly at http://localhost:8000/health
- [ ] T013 [US1] Test backend container restart without data loss by stopping and restarting container
- [ ] T014 [US1] Verify backend container logs are accessible via "docker logs todo-backend" (SC-010)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently. Backend container builds, runs, and serves API on port 8000.

---

## Phase 3: User Story 2 - Frontend Service Containerization (Priority: P2)

**Goal**: Package Next.js frontend into optimized multi-stage container image with nginx:alpine runtime

**Independent Test**: Build frontend container image, run locally, verify web interface loads and API proxy works to backend service

### Implementation for User Story 2

- [ ] T015 [P] [US2] Create frontend-app/Dockerfile with multi-stage build structure (Stage 1: node:18-alpine builder, Stage 2: nginx:alpine runtime)
- [ ] T016 [P] [US2] Create frontend-app/nginx.conf with static file serving, /api reverse proxy to http://todo-backend:8000, and /health endpoint
- [ ] T017 [US2] Implement Stage 1 (builder) in frontend-app/Dockerfile: set WORKDIR /app, copy package files, run npm ci, copy source, run npm run build
- [ ] T018 [US2] Implement Stage 2 (runtime) in frontend-app/Dockerfile: copy build artifacts from builder stage to /usr/share/nginx/html, copy nginx.conf, expose port 80
- [ ] T019 [US2] Build frontend container image using "docker build -t todo-frontend:latest ./frontend-app"
- [ ] T020 [US2] Verify frontend image size is under 100MB using "docker images" command (SC-004)
- [ ] T021 [US2] Run frontend container locally and verify startup time under 5 seconds (SC-002)
- [ ] T022 [US2] Test frontend container serves static files correctly at http://localhost:80
- [ ] T023 [US2] Test frontend container health check endpoint responds at http://localhost:80/health
- [ ] T024 [US2] Test frontend container API proxy by running both containers and verifying /api requests reach backend
- [ ] T025 [US2] Verify frontend container logs are accessible via "docker logs todo-frontend" (SC-010)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Frontend container builds, runs, serves UI on port 80, and proxies API calls to backend.

---

## Phase 4: User Story 3 - Container Image Optimization (Priority: P3)

**Goal**: Optimize container images for size and security to meet production standards

**Independent Test**: Run security scans on both images, verify sizes under targets, confirm no critical/high vulnerabilities

### Implementation for User Story 3

- [ ] T026 [P] [US3] Measure backend container image size and document current size in bytes
- [ ] T027 [P] [US3] Measure frontend container image size and document current size in bytes
- [ ] T028 [P] [US3] Run security vulnerability scan on backend image using Docker Scout or Trivy
- [ ] T029 [P] [US3] Run security vulnerability scan on frontend image using Docker Scout or Trivy
- [ ] T030 [US3] Review security scan results and verify zero critical or high-severity vulnerabilities (SC-007)
- [ ] T031 [US3] If vulnerabilities found, update base images or dependencies and rebuild
- [ ] T032 [US3] Verify multi-stage build for frontend excludes build tools from final image by inspecting layers
- [ ] T033 [US3] Verify backend image uses minimal base (python:3.11-slim) and frontend uses minimal base (nginx:alpine)
- [ ] T034 [US3] Document final image sizes and confirm backend <500MB and frontend <100MB

**Checkpoint**: All user stories should now be independently functional with optimized, secure container images.

---

## Phase 5: Integration Testing & Validation

**Purpose**: Verify both containers work together and meet all success criteria

- [ ] T035 [P] Run both backend and frontend containers together using docker run with --link
- [ ] T036 [P] Test complete user flow: access frontend, make API calls through proxy, verify backend processes requests
- [ ] T037 [P] Verify application functionality is identical between containerized and non-containerized deployments (SC-008)
- [ ] T038 [P] Test container restart scenario: stop both containers, restart, verify no data loss (SC-005)
- [ ] T039 [P] Measure combined build time for both images and verify under 5 minutes with caching (SC-006)
- [ ] T040 [P] Test containers with different runtimes (Docker, containerd if available) to verify OCI compliance (SC-009)

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Documentation, cleanup, and final verification

- [ ] T041 [P] Create or update backend/README.md with container build and run instructions
- [ ] T042 [P] Create or update frontend-app/README.md with container build and run instructions
- [ ] T043 [P] Verify all environment variables are documented in both Dockerfiles as comments
- [ ] T044 [P] Verify no secrets are hardcoded in Dockerfiles by inspecting image layers
- [ ] T045 [P] Test container startup with missing environment variables to verify graceful failure
- [ ] T046 Run final validation checklist from specs/009-containerization/quickstart.md
- [ ] T047 Document any deviations from plan or lessons learned in specs/009-containerization/IMPLEMENTATION_NOTES.md

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **User Story 1 (Phase 2)**: Depends on Setup completion - Backend containerization
- **User Story 2 (Phase 3)**: Depends on Setup completion - Frontend containerization (can run parallel to US1 for development, but testing requires US1 backend)
- **User Story 3 (Phase 4)**: Depends on US1 and US2 completion - Optimization
- **Integration Testing (Phase 5)**: Depends on US1, US2, US3 completion
- **Polish (Phase 6)**: Depends on all previous phases

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Setup - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Setup - Independent development, but integration testing requires US1 backend running
- **User Story 3 (P3)**: Depends on US1 and US2 - Requires both images to be built

### Within Each User Story

- Setup tasks (T001-T002) can run in parallel
- US1 tasks are sequential (read → modify → build → test)
- US2 tasks T015-T016 can run in parallel (Dockerfile and nginx.conf creation), then sequential for build and test
- US3 tasks T026-T029 can run in parallel (measurements and scans), then sequential for remediation
- Integration tasks can run in parallel
- Polish tasks can run in parallel

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- US1 and US2 can be developed in parallel (different files, no conflicts)
- US3 measurement and scanning tasks can run in parallel
- Integration testing tasks can run in parallel
- All Polish tasks can run in parallel

---

## Parallel Example: User Story 2

```bash
# Launch Dockerfile and nginx.conf creation together:
Task: "Create frontend-app/Dockerfile with multi-stage build structure"
Task: "Create frontend-app/nginx.conf with static file serving and API proxy"

# After both complete, proceed with build and test sequentially
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: User Story 1 (Backend containerization)
3. **STOP and VALIDATE**: Test backend container independently
4. Verify backend meets all acceptance criteria
5. Deploy/demo backend container if ready

### Incremental Delivery

1. Complete Setup → Foundation ready
2. Add User Story 1 → Test independently → Backend container working (MVP!)
3. Add User Story 2 → Test independently → Frontend container working
4. Add User Story 3 → Test independently → Optimized images
5. Integration testing → Full containerized application
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup together
2. Once Setup is done:
   - Developer A: User Story 1 (Backend)
   - Developer B: User Story 2 (Frontend)
3. After US1 and US2 complete:
   - Developer A or B: User Story 3 (Optimization)
4. Team: Integration testing together

---

## Gordon (Docker AI) Consultation Points

**Note**: Gordon consultations are recommended but not required tasks. Consult Gordon for expert guidance on Dockerfile optimization and security.

### Consultation 1: Backend Optimization (Before T004)
**Prompt**: "Analyze this Python FastAPI Dockerfile for security vulnerabilities and suggest optimizations to reduce image size below 500MB while maintaining Python 3.11-slim base."
**Input**: Current backend/Dockerfile content
**Use output for**: Tasks T004-T008 (backend Dockerfile modifications)

### Consultation 2: Frontend Dockerfile Generation (Before T015)
**Prompt**: "Generate a production-ready multi-stage Dockerfile for Next.js 14 with App Router. Stage 1: Build with Node.js 18. Stage 2: Serve with nginx:alpine. Target image size under 100MB."
**Use output for**: Tasks T015, T017-T018 (frontend Dockerfile creation)

### Consultation 3: nginx Configuration (Before T016)
**Prompt**: "Create an nginx.conf for serving Next.js static export with reverse proxy for /api requests to backend service at http://todo-backend:8000. Include health check endpoint."
**Use output for**: Task T016 (nginx.conf creation)

### Consultation 4: Security Scan (During T028-T029)
**Prompt**: "Scan both todo-backend and todo-frontend Docker images for security vulnerabilities. Report any critical or high-severity issues."
**Use output for**: Tasks T030-T031 (vulnerability remediation)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Backend Dockerfile already exists - tasks focus on optimization
- Frontend Dockerfile does not exist - tasks create from scratch
- No tests requested in specification - focus on build, run, verify
- Commit after each user story phase completion
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Success Criteria Verification

| Criterion | Verification Task | Expected Result |
|-----------|-------------------|-----------------|
| SC-001: Backend starts <10s | T011 | Container starts and responds within 10 seconds |
| SC-002: Frontend starts <5s | T021 | Container starts and serves within 5 seconds |
| SC-003: Backend image <500MB | T010, T026 | Image size under 500MB |
| SC-004: Frontend image <100MB | T020, T027 | Image size under 100MB |
| SC-005: Restart without data loss | T013, T038 | No data loss after restart |
| SC-006: Build <5 minutes | T039 | Combined build time under 5 minutes |
| SC-007: Zero critical vulnerabilities | T028-T030 | Security scans pass |
| SC-008: Functional parity | T037 | All features work in containers |
| SC-009: Runtime compatibility | T040 | Works with Docker, containerd |
| SC-010: Log accessibility | T014, T025 | Logs accessible via docker logs |

---

## Task Count Summary

- **Total Tasks**: 47
- **Setup Phase**: 2 tasks
- **User Story 1 (Backend)**: 12 tasks
- **User Story 2 (Frontend)**: 11 tasks
- **User Story 3 (Optimization)**: 9 tasks
- **Integration Testing**: 6 tasks
- **Polish**: 7 tasks

**Parallel Opportunities**: 18 tasks marked [P] can run in parallel within their phases

**Suggested MVP Scope**: Setup + User Story 1 (14 tasks) = Functional backend container
