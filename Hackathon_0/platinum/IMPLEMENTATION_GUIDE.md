# Platinum Tier: AI Employee - Implementation Guide

This guide provides a comprehensive, in-depth explanation of the Platinum Tier AI Employee system. It covers the architecture, detailed setup for various components, operational workflows, security best practices, troubleshooting, and extension points. This document assumes you have reviewed the high-level `README.md` and completed the initial setup outlined in `QUICK_START.md`.

---

## Table of Contents

1.  [Architectural Overview & Core Concepts](#1-architectural-overview--core-concepts)
    *   [Work-Zone Specialization](#work-zone-specialization)
    *   [Vault as the Central Hub](#vault-as-the-central-hub)
    *   [Task Lifecycle and Approval Workflow](#task-lifecycle-and-approval-workflow)
    *   [Multi-Agent Coordination (Claim-by-Move)](#multi-agent-coordination-claim-by-move)
    *   [Git-Based Vault Synchronization](#git-based-vault-synchronization)
2.  [Environment Configuration (.env)](#2-environment-configuration-env)
    *   [General Settings](#general-settings)
    *   [Vault Sync Configuration](#vault-sync-configuration)
    *   [Cloud Secrets](#cloud-secrets)
    *   [AI Content Generation](#ai-content-generation)
    *   [Odoo Configuration](#odoo-configuration)
    *   [Local Secrets](#local-secrets)
    *   [Security Settings](#security-settings)
    *   [Monitoring & Alerting](#monitoring--alerting)
    *   [Cloud VM Settings (for Local Reference)](#cloud-vm-settings-for-local-reference)
    *   [Docker Settings](#docker-settings)
3.  [Cloud VM Deployment & Setup](#3-cloud-vm-deployment--setup)
    *   [Deployment Script (`deploy-to-cloud.sh`)](#deployment-script-deploy-to-cloudsh)
    *   [Cloud-Init Configuration (`cloud/cloud-init.yaml`)](#cloud-init-configuration-cloudcloud-inityaml)
    *   [Standalone Setup Script (`cloud/setup-cloud-vm.sh`)](#standalone-setup-script-cloudsetup-cloud-vmsh)
    *   [Post-Deployment Steps](#post-deployment-steps)
4.  [Dockerized Environment](#4-dockerized-environment)
    *   [Project Root `docker-compose.yml`](#project-root-docker-composeyml)
    *   [Cloud Agent Dockerfile (`docker/cloud-agent/Dockerfile`)](#cloud-agent-dockerfile-dockercloud-agentdockerfile)
    *   [Vault Sync Dockerfile (`docker/vault-sync/Dockerfile`)](#vault-sync-dockerfile-dockervault-syncdockerfile)
5.  [Agent Implementations](#5-agent-implementations)
    *   [Cloud Agent (`cloud/agents/cloud_agent_fixed.py`)](#cloud-agent-cloudagentscloud_agent_fixedpy)
    *   [Local Agent (`local/agents/local_agent_fixed.py`)](#local-agent-localagentslocal_agent_fixedpy)
6.  [Gmail API Integration](#6-gmail-api-integration)
    *   [Authentication Process](#authentication-process)
    *   [Required Files: `credentials.json` & `gmail_token.json`](#required-files-credentialsjson--gmail_tokenjson)
    *   [Gmail Scopes & Security](#gmail-scopes--security)
7.  [Odoo Accounting Integration](#7-odoo-accounting-integration)
    *   [Odoo Tools & Workflows](#odoo-tools--workflows)
    *   [Direct Integration vs. MCP Server](#direct-integration-vs-mcp-server)
8.  [AI Content Generation System](#8-ai-content-generation-system)
    *   [Gemini AI Setup & Configuration](#gemini-ai-setup--configuration)
    *   [Core Generator (`gemini_content_generator.py`)](#core-generator-gemini_content_generatorpy)
    *   [Single Post Generation (`generate_post.py`)](#single-post-generation-generate_postpy)
    *   [Batch Content Generation (`batch_content_generator.py`)](#batch-content-generation-batch_content_generatorpy)
    *   [Testing AI Features (`test_gemini_ai.py`)](#testing-ai-features-test_gemini_aipy)
9.  [Social Media Integrations](#9-social-media-integrations)
    *   [Instagram Posting (with Images via ngrok)](#instagram-posting-with-images-via-ngrok)
    *   [LinkedIn Posting (via Playwright)](#linkedin-posting-via-playwright)
    *   [Facebook & Twitter Posting](#facebook--twitter-posting)
10. [Monitoring, Logging & Backup](#10-monitoring-logging--backup)
    *   [Health Monitoring Script (`health-monitor.sh`)](#health-monitoring-script-health-monitorsh)
    *   [Odoo Database Backup (`backup-odoo.sh`)](#odoo-database-backup-backup-odoosh)
    *   [Log Rotation](#log-rotation)
11. [Testing & Demo Workflow](#11-testing--demo-workflow)
    *   [Comprehensive Test Suite (`test-platinum.py`)](#comprehensive-test-suite-test-platinumpy)
    *   [Platinum Demo (`platinum-demo.py`)](#platinum-demo-platinum-demopy)
12. [Troubleshooting & Maintenance](#12-troubleshooting--maintenance)

---

## 1. Architectural Overview & Core Concepts

The Platinum Tier AI Employee operates on a robust **Cloud-Local Hybrid Architecture**, designed for 24/7 autonomous operation with human oversight for critical approvals and sensitive actions. The core principle is **Work-Zone Specialization**, where Cloud Agents handle drafting and initiation, and Local Agents manage approvals and final execution. The **Vault**, a Git-managed directory, serves as the central communication hub.

### Work-Zone Specialization

Tasks are divided based on sensitivity and execution environment:

| Domain        | Cloud Responsibilities                       | Local Responsibilities                               |
| :------------ | :------------------------------------------- | :--------------------------------------------------- |
| **Email**     | Triage, draft replies                        | Review, approve, send (via real Gmail API)           |
| **Social Media** | Draft posts, generate AI content, schedule drafts | Review, approve, publish (via real social APIs)     |
| **Accounting (Odoo)** | Draft invoices, draft payments                | Review, approve, post invoices/payments (to Odoo) |
| **WhatsApp**  | ❌ Never touches session                    | ✅ Full session control (secure, local only)         |
| **Banking**   | ❌ No credentials                           | ✅ All credentials (secure, local only)              |
| **Payments**  | ❌ Never executes                           | ✅ Final execution (secure, local only)              |

This specialization ensures sensitive credentials and final execution logic reside solely on the local, human-controlled machine, while the cloud handles continuous, less sensitive drafting and monitoring.

### Vault as the Central Hub

The `/vault` directory is the heart of the system's communication. It is a Git repository synchronized between the Cloud VM and the Local machine. Agents communicate by creating, moving, and modifying markdown files within specific subdirectories of the vault.

Key vault subdirectories and their roles:
*   `/Needs_Action/`: Tasks initiated by agents (e.g., Cloud Agent detecting a new email).
    *   `/Needs_Action/email/`: Markdown files representing new emails to be processed.
    *   `/Needs_Action/social/`: Markdown files representing social media tasks.
    *   `/Needs_Action/odoo/`: Markdown files representing Odoo-related tasks.
*   `/In_Progress/<agent_name>/`: Tasks currently being processed by a specific agent.
*   `/Pending_Approval/`: Drafts or actions generated by the Cloud Agent that require human review and approval.
*   `/Approved/`: Tasks moved here by the human operator, signaling approval for the Local Agent to execute.
*   `/Done/<agent_name>/`: Completed tasks.
*   `/Updates/`: Temporary directory used by agents to signal new activity, which then gets synced.
*   `/Logs/`: Stores execution logs from both agents.

### Task Lifecycle and Approval Workflow

A typical task follows this lifecycle:

1.  **Initiation (Cloud Agent)**: A Cloud Agent watcher (e.g., Gmail watcher) detects an event.
2.  **Task Creation (`Needs_Action`)**: The Cloud Agent creates a markdown file in `/vault/Needs_Action/` (e.g., `EMAIL_*.md`).
3.  **Claiming (`In_Progress`)**: The Cloud Agent claims the task by moving it to `/vault/In_Progress/cloud_agent/`.
4.  **Drafting & Approval Request**: The Cloud Agent processes the task (e.g., drafts an email reply or social post) and creates a markdown file in `/vault/Pending_Approval/` (e.g., `EMAIL_SEND_*.md`, `AI_SOCIAL_*.md`). This file contains the draft content and metadata for Local Agent execution.
5.  **Cloud Sync**: The Cloud Agent commits and pushes these changes to the remote Git vault.
6.  **Local Sync**: The Local Agent pulls updates from the remote Git vault, receiving the new approval request.
7.  **Human Review/Approval**: The human operator reviews the `Pending_Approval` file. To approve, they move it to `/vault/Approved/`.
8.  **Execution (Local Agent)**: The Local Agent detects the approved file in `/vault/Approved/`, processes its metadata, and executes the final action (e.g., sends the email via Gmail API, publishes the social post).
9.  **Completion (`Done`)**: The Local Agent moves the executed file to `/vault/Done/local/` and logs the execution.
10. **Local Sync**: The Local Agent commits and pushes these completion updates to the remote Git vault.

### Multi-Agent Coordination (Claim-by-Move)

The `shared/claim_manager/claim_manager.py` module enforces a "claim-by-move" rule. This mechanism prevents race conditions and ensures that tasks are processed exclusively by a single agent at a time:

*   **Claiming:** When an agent (Cloud or Local) identifies a task in `/vault/Needs_Action/`, it attempts to "claim" it by atomically moving the corresponding markdown file to its specific `/vault/In_Progress/<agent_name>/` folder.
*   **Exclusivity:** The `ClaimManager` checks if any other agent has already claimed the task. If a task is already in *any* `/In_Progress/` subfolder, it cannot be claimed again.
*   **Releasing:** Once an agent completes processing, it moves the task file from its `/In_Progress/` folder to a final state folder (`/Done/`, `/Rejected/`, or `/Failed/`).
*   **Robustness:** A `cleanup_stale_claims` function (though not actively called in the main loops of the agents by default, it exists as a utility) is available to move tasks back to `/Needs_Action/` if they remain in `/In_Progress/` for too long (e.g., due to an agent crash), allowing other agents to re-attempt processing.

### Git-Based Vault Synchronization

The `shared/vault_sync/git_sync.py` module implements a robust Git-based synchronization mechanism for the `/vault` directory. This ensures consistency of tasks and state across all connected environments.

*   **Initialization:** The `VaultSync` class can initialize a new Git repository in `/vault`, clone from a remote URL, or use an existing repository. It configures basic Git user details.
*   **Security (`.gitignore`):** A critical aspect is the hardcoded `.gitignore` created *within* the `/vault` directory during initialization. This file explicitly prevents sensitive data from being committed to the Git repository, including:
    *   `.env` files (e.g., `vault/.env`)
    *   `secrets/`, `tokens/`, `credentials/` directories
    *   `*.key`, `*.pem` files
    *   `*.session`, `whatsapp_session/`, `session_data/` (for local-only session data)
    *   Various log files and temporary files.
    This reinforces the principle of "NO secrets (.env, tokens, sessions never sync)" to the cloud.
*   **Synchronization Operations:**
    *   `push_updates()`: Adds all changes in the vault, commits them, and pushes to the configured remote Git repository. Primarily used by the Cloud Agent after generating drafts/approvals, and by the Local Agent after executing approvals.
    *   `pull_updates()`: Fetches and performs a Git rebase from the remote repository. This keeps the local history clean and integrates changes from other environments. Includes basic conflict handling.
    *   `force_sync()`: Resets the local vault to match the remote's state, useful for resolving unmanageable conflicts.
*   **Environment Variables:** Relies on `GIT_REPO_URL` and `GIT_EMAIL` for configuration. `SYNC_INTERVAL` is used by the `SyncScheduler` (though agents call push/pull directly in their loops).

---

## 2. Environment Configuration (.env)

The `.env` file is central to configuring the Platinum AI Employee. It defines critical environment variables for agents, integrations, and operational settings. A template, `.env.example`, is provided, which should be copied to `.env` and filled with your specific values.

**SECURITY WARNING:** The `.env` file contains sensitive credentials. It **MUST NEVER** be committed to version control. Ensure it is excluded by your project's root `.gitignore` (which is configured to do so by default).

### General Settings

*   **`ENVIRONMENT`**: `local` or `cloud`. Determines the agent's operating context.
*   **`AGENT_ROLE`**: `drafts_only` (for Cloud Agent) or `approvals_and_execution` (for Local Agent). This configures the agent's primary function according to work-zone specialization.

### Vault Sync Configuration

Used by `shared/vault_sync/git_sync.py` and potentially embedded in `docker-compose.yml`.

*   **`GIT_REPO_URL`**: The URL of the Git repository used for vault synchronization. This should be a private repository.
*   **`GIT_EMAIL`**: Email address used for Git commits by the AI Employee (e.g., `ai-employee@local`).
*   **`VAULT_PATH`**: Path to the local `vault` directory. (Defaults to `./vault`).
*   **`SYNC_INTERVAL`**: How often the `vault-sync` container (or agent loops) attempts to pull/push updates (in seconds, default: 300s).

### Cloud Secrets (Cloud VM only)

These credentials are **ONLY** required on the Cloud VM. They should **NOT** be present in the `.env` file on your Local machine, nor should they be synced to the vault.

*   **Gmail API**:
    *   `GMAIL_CLIENT_ID`
    *   `GMAIL_CLIENT_SECRET`
    *   `GMAIL_REFRESH_TOKEN` (Generated after first authentication)
    *   `GMAIL_REDIRECT_URI` (Typically `http://localhost:8080/callback`)
*   **Social Media**:
    *   `FACEBOOK_ACCESS_TOKEN`
    *   `FACEBOOK_PAGE_ID`
    *   `INSTAGRAM_ACCESS_TOKEN`
    *   `INSTAGRAM_ACCOUNT_ID`
    *   `TWITTER_BEARER_TOKEN` (for Twitter API v2)
    *   `TWITTER_API_KEY`, `TWITTER_API_SECRET`, `TWITTER_ACCESS_TOKEN`, `TWITTER_ACCESS_SECRET` (for older Twitter APIs, if used)
    *   `LINKEDIN_ACCESS_TOKEN`

### AI Content Generation

*   **`GEMINI_API_KEY`**: Your Google Gemini API key. Obtained from [https://aistudio.google.com/app/apikey](https://aistudio.google.com/app/apikey).
*   **`GEMINI_MODEL`**: The Gemini model to use (e.g., `gemini-2.5-flash`, `gemini-2.0-pro`). `gemini-2.5-flash` is recommended for its speed and free tier.

### Odoo Configuration

These are typically used by both Cloud (for drafting) and Local (for execution) agents, or by the Odoo Docker service.

*   **`ODOO_URL`**: URL of the Odoo instance (e.g., `http://localhost:8069` for Docker, or the VM's internal IP).
*   **`ODOO_DB`**: Odoo database name (default: `odoo`).
*   **`ODOO_USERNAME`**: Odoo user for API access (default: `admin`).
*   **`ODOO_PASSWORD`**: Odoo user password (default: `admin`). **CHANGE THIS!**
*   **`ODOO_DB_PASSWORD`**: PostgreSQL password for the Odoo database (used in Docker Compose, default: `odoo`). **CHANGE THIS!**

### Local Secrets (Local machine only)

These credentials are **ONLY** required on the Local machine. They should **NOT** be present in the `.env` file on the Cloud VM, nor should they be synced to the vault.

*   **WhatsApp Session**:
    *   `WHATSAPP_SESSION_PATH`: Local path to store WhatsApp session data.
    *   `WHATSAPP_PHONE`: WhatsApp phone number.
*   **Banking Credentials**:
    *   `BANKING_API_KEY`
    *   `BANKING_USERNAME`
    *   `BANKING_PASSWORD`
    *   `PAYMENT_TOKEN`
    *   `PAYMENT_GATEWAY_URL`
*   **Local MCP Credentials**:
    *   `LOCAL_MCP_API_KEY`

### Security Settings

*   **`DRY_RUN`**: `true` or `false`. When `true`, agents will simulate actions without actual execution (e.g., not sending real emails). Useful for testing.
*   **`MAX_PAYMENT_AMOUNT`**: Maximum payment amount (in currency units) that can be automatically executed without additional approval.
*   **`APPROVAL_TIMEOUT`**: Time (in seconds) before an approval request might expire or be flagged (default: 86400s = 24 hours).

### Monitoring & Alerting

*   **`ALERT_EMAIL`**: Email address to send health alerts to (used by `health-monitor.sh`).
*   **`HEALTH_CHECK_INTERVAL`**: How often health checks run (in seconds).
*   **`LOG_LEVEL`**: Logging verbosity (DEBUG, INFO, WARNING, ERROR, CRITICAL).

### Cloud VM Settings (for Local Reference)

These are for informational purposes on the Local machine to connect to the Cloud VM.

*   **`CLOUD_VM_IP`**: Public IP address of the Cloud VM.
*   **`CLOUD_VM_SSH_KEY`**: Path to the SSH private key for connecting to the Cloud VM (e.g., `~/.ssh/id_rsa`).
*   **`CLOUD_VM_USER`**: SSH username for the Cloud VM (e.g., `ai-employee`, `ubuntu`).

### Docker Settings

*   **`DOCKER_COMPOSE_PROJECT`**: Name used for the Docker Compose project (default: `platinum-ai-employee`).

---

## 3. Cloud VM Deployment & Setup

Deploying the Platinum AI Employee to a Cloud Virtual Machine (VM) enables 24/7 autonomous operations. This section details the automated deployment scripts and the underlying configuration.

### Deployment Script (`scripts/deploy-to-cloud.sh`)

This shell script automates the provisioning of a cloud VM and its initial setup.

**Usage:**
```bash
./scripts/deploy-to-cloud.sh
```

**Workflow:**
1.  **Provider Selection**: Prompts the user to choose between Oracle Cloud Free Tier, AWS EC2, or a Manual SSH Setup.
2.  **VM Provisioning**:
    *   **Oracle Cloud**: Uses the `oci` CLI to launch a `VM.Standard.A1.Flex` ARM instance. It requires `COMPARTMENT_ID`, `AVAILABILITY_DOMAIN`, `SUBNET_ID`, and the path to your **SSH Public Key**. It passes the content of `cloud/cloud-init.yaml` as `user_data` for initial VM configuration.
    *   **AWS EC2**: Uses the `aws` CLI to launch a `t3.medium` instance. It requires `AMI_ID`, `KEY_NAME`, `SECURITY_GROUP_ID`, and `SUBNET_ID`.
    *   **Manual SSH**: Requires the VM's public IP and SSH username.
3.  **Initial Setup on VM**: After provisioning (or for manual setup), the script connects to the VM via SSH and executes `cloud/setup-cloud-vm.sh` on the remote machine. This script handles the installation of Docker, Git, Python, Nginx, and other dependencies, along with setting up the project's directory structure and Docker Compose services.
4.  **Output**: Provides the VM's public IP and critical next steps.

**Prerequisites:**
*   An active account with Oracle Cloud Infrastructure (OCI) or Amazon Web Services (AWS).
*   `oci` CLI installed and configured for Oracle Cloud.
*   `aws` CLI installed and configured for AWS.
*   An SSH key pair for secure access to the VM.
*   Your local machine must have `ssh` and `scp` commands available.

### Cloud-Init Configuration (`cloud/cloud-init.yaml`)

This declarative configuration is specifically used for **Oracle Cloud** deployments via `deploy-to-cloud.sh` to configure the VM on first boot.

**Key Configurations:**
*   **Package Installation**: Installs `docker.io`, `docker-compose`, `python3`, `git`, `nginx`, `certbot`, `mailutils`, and other utilities.
*   **User Management**: Creates an `ai-employee` user, adds it to `sudo` and `docker` groups, and injects the specified SSH public key for remote access.
*   **File Writing**: Embeds the following configurations directly onto the VM:
    *   `/opt/ai-employee/docker-compose.yml`: Defines Docker Compose services for `cloud-agent`, `odoo`, `db`, `health-monitor` (Nginx), and `vault-sync`. Environment variables for these services are referenced (`${VAR_NAME}`), expecting them to be defined in `/opt/ai-employee/.env` later.
    *   `/opt/ai-employee/nginx.conf`: Configures Nginx for the `/health` endpoint and HTTP to HTTPS redirection.
    *   `/opt/ai-employee/health.json`: A simple JSON file for the health endpoint.
    *   `/etc/systemd/system/cloud-agent.service`: A Systemd service to manage `docker-compose up -d` and `docker-compose down` for the AI Employee services.
*   **Command Execution (`runcmd`)**:
    *   Enables and starts Docker and the `cloud-agent` Systemd service.
    *   Creates the full `/opt/ai-employee` directory structure, including all `vault/` subfolders.
    *   Sets up `logrotate` for `/opt/ai-employee/logs/`.
    *   Configures automated package updates.
    *   Adds a cron job for Odoo database backups (`backup-odoo.sh`).

### Standalone Setup Script (`cloud/setup-cloud-vm.sh`)

This script is executed on the remote Cloud VM. It is used by `deploy-to-cloud.sh` (for AWS and Manual SSH options) or can be run manually. It provides a robust, imperative setup process.

**Key Actions:**
*   **Root Execution**: Requires root privileges to run.
*   **Dependency Installation**: Updates `apt` and installs the same essential packages as `cloud-init.yaml`.
*   **User Management**: Creates and configures the `ai-employee` user.
*   **Directory Structure**: Creates the complete `/opt/ai-employee` project and `vault/` subdirectories.
*   **Embedded Configurations**: Uses `cat` to write the following files to `/opt/ai-employee`:
    *   `docker-compose.yml`: Defines the Docker services (similar to `cloud-init.yaml`, but notably does *not* include the `health-monitor` service directly, instead setting up a separate cron-driven health script).
    *   `nginx.conf`: Nginx configuration.
    *   `health.json`: Simple health endpoint.
    *   `scripts/backup-odoo.sh`: Script for Odoo database and filestore backup.
    *   `scripts/health-monitor.sh`: Script for periodic health checks of services, containers, disk, and memory, with optional email alerts.
*   **Service & Cron Job Setup**:
    *   Configures and enables the `cloud-agent` Systemd service.
    *   Sets up `logrotate`.
    *   Adds cron jobs for `backup-odoo.sh` and `health-monitor.sh`.

### Post-Deployment Steps

After the VM is provisioned and initial setup is complete, you must perform the following:

1.  **Copy `.env` file**: Securely copy your Cloud-specific `.env` file to the VM.
    ```bash
    scp /path/to/your/cloud/.env ai-employee@<CLOUD_VM_IP>:/opt/ai-employee/.env
    ```
2.  **Clone the project repository**:
    ```bash
    ssh ai-employee@<CLOUD_VM_IP> 'cd /opt/ai-employee && git clone <your-repo-url> .'
    ```
3.  **Start Cloud Agent services**:
    ```bash
    ssh ai-employee@<CLOUD_VM_IP> 'sudo systemctl start cloud-agent'
    ```
4.  **Initialize Vault Sync**:
    ```bash
    ssh ai-employee@<CLOUD_VM_IP> 'cd /opt/ai-employee && ./scripts/setup-vault-sync.sh'
    ```
    (Ensure you provide the same `GIT_REPO_URL` as configured on your Local machine).
5.  **Check Cloud health**:
    ```bash
    curl http://<CLOUD_VM_IP>/health
    ```

---

## 4. Dockerized Environment

The Platinum Tier can be deployed using Docker and Docker Compose, both locally and on the Cloud VM (as seen in `cloud-init.yaml` and `setup-cloud-vm.sh`). This section details the primary `docker-compose.yml` for local development and the Dockerfiles for the agent services.

### Project Root `docker-compose.yml`

This `docker-compose.yml` (located in the project root) is primarily used for running the system locally.

**Services:**
*   **`odoo`**: Runs the Odoo ERP system (version 17.0).
    *   **Ports**: Exposed on host port `8069`.
    *   **Dependencies**: Relies on the `db` service.
    *   **Environment**: Configured with `HOST`, `USER`, `PASSWORD` for Odoo. `ODOO_DB_PASSWORD` is injected from the host's `.env`.
    *   **Volumes**: `platinum-odoo-web-data` (persistent Odoo data), `./odoo/addons` (mounts custom Odoo modules), `./odoo/logs` (mounts host logs).
*   **`db`**: Runs a PostgreSQL database (version 15) for Odoo.
    *   **Environment**: `POSTGRES_DB`, `POSTGRES_USER`, `POSTGRES_PASSWORD` are injected from `.env`.
    *   **Volumes**: `platinum-odoo-db-data` for persistent database storage.
*   **`cloud-agent`**: Runs the Cloud Agent.
    *   **Build Context**: Built from `./docker/cloud-agent/Dockerfile`.
    *   **Dependencies**: Depends on `odoo`.
    *   **Environment**: A comprehensive set of environment variables from the host's `.env` are passed, including all Gmail, Odoo, Social Media, Gemini AI, and Git configurations.
    *   **Volumes**:
        *   `platinum-vault:/app/vault`: **Crucially, this bind-mounts the host's `./vault` directory directly into the container.** This allows the Cloud Agent running inside Docker to interact with the Git-managed vault on the host filesystem.
        *   `./cloud:/app/cloud:ro`: Mounts the Cloud Agent's code read-only.
        *   `./shared:/app/shared:ro`: Mounts shared code (e.g., `ClaimManager`, `VaultSync`) read-only.
        *   `./logs/cloud-agent:/app/logs`: Mounts host directory for container logs.
    *   **Command**: Executes `python cloud/agents/cloud_agent_fixed.py`.
*   **`vault-sync`**: Runs the Vault Synchronization component.
    *   **Build Context**: Built from `./docker/vault-sync/Dockerfile`.
    *   **Dependencies**: Depends on `cloud-agent`.
    *   **Environment**: `GIT_REPO_URL`, `GIT_EMAIL`, `SYNC_INTERVAL`, `VAULT_PATH` are passed from the host's `.env`.
    *   **Volumes**: `platinum-vault:/app/vault` (bind-mounts the host's vault, similar to `cloud-agent`).
    *   **Command**: `python -c "from shared.vault_sync.git_sync import SyncScheduler, VaultSync; ... scheduler.start()"`. This directly launches the `SyncScheduler` to continuously pull and push vault updates.
*   **Networks**: All services communicate over a `platinum-network` bridge network.
*   **Volumes (`platinum-vault`)**: The special `platinum-vault` volume configured with `type: none`, `o: bind`, `device: ./vault` ensures that the Git-managed vault on your host machine is directly shared with the Docker containers, enabling seamless Git operations by the agents.

### Cloud Agent Dockerfile (`docker/cloud-agent/Dockerfile`)

This `Dockerfile` defines the build process for the `cloud-agent` Docker image.

*   **Base Image**: `python:3.12-slim` for a lightweight Python environment.
*   **System Dependencies**: Installs `git` and `curl`. `git` is essential for the `VaultSync` operations if the agent were to perform them directly, though in the orchestrated setup, `vault-sync` container handles this.
*   **Python Dependencies**: Copies `requirements.txt` from the project root and installs all Python packages via `pip`.
*   **Application Code**: Copies the `cloud/` and `shared/` directories into the container.
*   **Health Check**: Defines an internal Docker `HEALTHCHECK` to ensure the Python process is alive.
*   **Entry Point**: `CMD ["python", "cloud/agents/cloud_agent_fixed.py"]` launches the Cloud Agent script.

### Vault Sync Dockerfile (`docker/vault-sync/Dockerfile`)

This `Dockerfile` defines the build process for the `vault-sync` Docker image.

*   **Base Image**: `python:3.12-slim` for consistency.
*   **System Dependencies**: Installs `git`, which is crucial as the `VaultSync` mechanism directly uses Git commands.
*   **Python Dependencies**: Copies `requirements.txt` and installs Python packages (including `GitPython`).
*   **Application Code**: Copies the `shared/` directory (containing `git_sync.py`) into the container.
*   **Entry Point**: The `CMD` is a placeholder; the actual command to launch the `SyncScheduler` is overridden in `docker-compose.yml`.

---

## 5. Agent Implementations

The core logic of the Platinum Tier AI Employee resides in its Cloud and Local agents, which adhere strictly to the principle of work-zone specialization.

### Cloud Agent (`cloud/agents/cloud_agent_fixed.py`)

The Cloud Agent is the **"Drafts Only"** agent, running 24/7 on the Cloud VM. Its primary responsibilities are to monitor external systems, initiate tasks, perform AI-driven drafting, and create approval requests for the Local Agent. It **NEVER** executes sensitive actions directly.

**Key Responsibilities & Workflows:**

*   **Monitoring**: Continuously checks external systems for events (e.g., unread, important emails via Gmail API watcher).
*   **Task Creation**: For detected events, it creates markdown files in `/vault/Needs_Action/` (e.g., `EMAIL_*.md`).
*   **Task Claiming**: Uses `ClaimManager` to claim tasks from `/vault/Needs_Action/` by moving them to `/vault/In_Progress/cloud_agent/`.
*   **Drafting Logic**:
    *   **Email Triage**: Analyzes email content (subject, body, sender) to determine if a reply is needed, categorizes it, and drafts a templated response.
    *   **AI Content Generation**: Acts as the primary entry point for AI social media content generation using Google Gemini. It utilizes `GeminiContentGenerator` and `AIContentWorkflow` to create posts based on topics, platforms, and styles.
    *   **Odoo Drafting**: Can draft Odoo-related tasks (invoices, payments) based on internal triggers or other system events.
*   **Approval Request Creation**: For all drafted content or proposed actions, the Cloud Agent creates a markdown file in `/vault/Pending_Approval/` (e.g., `EMAIL_SEND_*.md`, `AI_SOCIAL_*.md`, `WEEKLY_SOCIAL_*.md`, `INVOICE_*.md`). These files contain the draft, metadata, and instructions for human review.
*   **Vault Synchronization**: After creating new tasks or approval requests, it uses `VaultSync` to commit and push changes to the remote Git vault.
*   **Gmail Scopes**: While its `_init_gmail` function includes both `gmail.readonly` and `gmail.send` scopes for token consistency (using the same `gmail_token.json` as Local), the Cloud Agent itself *only* uses `gmail.readonly` to fetch emails. It does not send emails.

### Local Agent (`local/agents/local_agent_fixed.py`)

The Local Agent is the **"Approvals + Final Execution"** agent, running on the Local machine. It serves as the human-in-the-loop, processing tasks requiring sensitive credentials or final human review before execution.

**Key Responsibilities & Workflows:**

*   **Vault Synchronization**: Continuously pulls updates from the remote Git vault using `VaultSync`, receiving new tasks and approval requests initiated by the Cloud Agent.
*   **Approval Processing**:
    *   Monitors `/vault/Pending_Approval/` for new approval requests. In an interactive mode, it can prompt the user.
    *   Actively monitors `/vault/Approved/` for files moved there by the human operator, signaling approval.
*   **Execution of Approved Actions**: Based on the `type` in the approval request's frontmatter, it executes the corresponding sensitive action:
    *   **Real Email Sending**: Uses the authenticated Gmail API (`_send_email_real`) to send emails as drafted by the Cloud Agent and approved by the human.
    *   **Real Social Media Posting**: Publishes content to Facebook, Instagram, Twitter, or LinkedIn.
        *   **Instagram**: Supports both `media_url` and `media_path`. For local image files (`media_path`), it dynamically starts a temporary local HTTP server and uses `pyngrok` to create a public tunnel, allowing Instagram's API to fetch the image.
        *   **LinkedIn**: Utilizes `Playwright` for browser automation to interact with the LinkedIn web interface, maintaining a persistent session in `/vault/linkedin_session/`.
    *   **Odoo Accounting**: Interacts directly with the Odoo instance via XML-RPC (`_post_invoice_odoo`, `_execute_payment_real`) to post invoices or record payments.
    *   **Banking/Payments**: Contains placeholders for actual banking API integration (`_execute_payment_real`), currently simulating payment recording in Odoo.
*   **Task Completion & Logging**: After successful execution, it moves the task file to `/vault/Done/local/` and records a detailed JSON log entry in `/vault/Logs/`.
*   **Security Context**: It stores and manages sensitive credentials (WhatsApp sessions, banking details, full Gmail API access) locally and never exposes them to the cloud.

---

## 6. Gmail API Integration

Gmail integration is fundamental for the AI Employee, enabling the Cloud Agent to read and triage emails, and the Local Agent to send replies. This requires a one-time OAuth 2.0 authentication process.

### Authentication Process

To grant the application access to your Gmail account, you must perform an OAuth 2.0 flow:

1.  **Prerequisites**: Ensure you have `credentials.json` (Google API client ID/secret) from your Google Cloud Console project. This file should be placed in your project root. If you are migrating from a Gold tier project, you can use `scripts/copy_credentials.bat` to copy it.
2.  **Run Authentication Script**:
    ```bash
    ./scripts/authenticate_gmail.bat
    ```
    This script executes an inline Python script that will:
    *   Open a browser window.
    *   Prompt you to sign in to your Google account and grant the necessary permissions.
3.  **Token Storage**: Upon successful authorization, an OAuth token (including a refresh token) will be saved to `gmail_token.json` in your project root. This `gmail_token.json` file is sensitive and allows programmatic access to your Gmail account.

### Required Files: `credentials.json` & `gmail_token.json`

*   **`credentials.json`**: Contains your Google API client ID and client secret. This file is obtained from the Google Cloud Console.
*   **`gmail_token.json`**: Contains the OAuth access and refresh tokens. This file is generated by the `authenticate_gmail.bat` script.

**SECURITY WARNING:** Both `credentials.json` and `gmail_token.json` are highly sensitive. They **MUST NEVER** be committed to version control. They are explicitly excluded by the root `.gitignore` and the vault's `.gitignore`. Keep these files secure on your local machine.

### Gmail Scopes & Security

The authentication process requests the following scopes:
*   `https://www.googleapis.com/auth/gmail.readonly` (for reading emails)
*   `https://www.googleapis.com/auth/gmail.send` (for sending emails)

**Security Implications**:
*   The **Cloud Agent** uses these scopes primarily for reading unread, important emails (`gmail.readonly`). Although `gmail.send` is included for token consistency, the Cloud Agent **never** sends emails directly. It only drafts replies and creates approval requests.
*   The **Local Agent** uses these scopes for sending approved email replies (`gmail.send`). Since `gmail_token.json` (and the ability to send emails) resides only on the Local machine, sensitive sending operations are under human control.

---

## 7. Odoo Accounting Integration

Odoo integration provides automated accounting operations for the AI Employee, from drafting invoices to recording payments.

### Odoo Tools & Workflows

*   **Cloud Agent (Drafting)**: Can initiate Odoo-related tasks (e.g., `invoice_draft`, `payment_draft`) by creating markdown files in `/vault/Needs_Action/` or directly in `/vault/Pending_Approval/`. These drafts are based on internal triggers or other system events.
*   **Local Agent (Execution)**: Monitors approved Odoo tasks in `/vault/Approved/`. Upon approval, it directly interacts with the configured Odoo instance via XML-RPC.
    *   **Invoice Posting**: Creates new invoices in Odoo, handling customer lookups (searching for existing partners or creating new ones).
    *   **Payment Recording**: Records payments in Odoo (currently a simulation, awaiting actual banking API integration).
*   **Financial Summary**: Odoo data (invoices, payments) is used to generate financial summaries, which can be integrated into reporting (e.g., a "Monday Morning CEO Briefing").

### Direct Integration vs. MCP Server

*   **Current Agent Integration**: The `local/agents/local_agent_fixed.py` directly uses Python's `xmlrpc.client` to communicate with the Odoo instance, configured via `.env` variables (`ODOO_URL`, `ODOO_DB`, `ODOO_USERNAME`, `ODOO_PASSWORD`).
*   **`odoo/mcp/odoo_server.py`**: This file contains a **standalone Odoo Micro-Agent Communication Protocol (MCP) server** which exposes various Odoo functionalities (get invoices, create invoice, get payments, financial summary) via a JSON-RPC interface. While it is a fully functional Odoo API server, it is **not currently integrated** as an intermediary between the agents and Odoo in the current Platinum Tier agent loops. The `LocalAgent` bypasses it and interacts directly. This `odoo_server.py` can be seen as an example of how to build a dedicated Odoo microservice or an alternative integration point for future extensions.

**Configuration:**
*   Ensure Odoo is running and accessible (e.g., `http://localhost:8069` if Dockerized).
*   Configure the following in your `.env` file (Cloud VM's `.env` for Cloud Agent, Local machine's `.env` for Local Agent):
    *   `ODOO_URL`
    *   `ODOO_DB`
    *   `ODOO_USERNAME` (Must have API access in Odoo)
    *   `ODOO_PASSWORD`
    *   `ODOO_DB_PASSWORD` (for the PostgreSQL database).

---

## 8. AI Content Generation System

The Platinum Tier leverages Google Gemini AI to autonomously generate a wide range of social media content, integrating it seamlessly into the approval workflow.

### Gemini AI Setup & Configuration

1.  **Get API Key**: Obtain a free `GEMINI_API_KEY` from [https://aistudio.google.com/app/apikey](https://aistudio.google.com/app/apikey).
2.  **Install SDK**: Install the Google Generative AI Python SDK:
    ```bash
    pip install google-generativeai
    ```
3.  **Configure `.env`**: Add your API key and chosen model to your `.env` file:
    ```env
    GEMINI_API_KEY=your_gemini_api_key_here
    GEMINI_MODEL=gemini-2.5-flash
    ```
    `gemini-2.5-flash` is recommended for its speed and free-tier limits (60 requests/minute, 1500 requests/day).
4.  **Verify Setup**: Run the test script to ensure everything is configured correctly:
    ```bash
    python social_platform/agents/test_gemini_ai.py
    ```

### Core Generator (`social_platform/agents/gemini_content_generator.py`)

This Python module contains the `GeminiContentGenerator` class, which directly interfaces with the Google Gemini API, and the `AIContentWorkflow` class, which integrates AI generation with the vault.

**`GeminiContentGenerator` Features:**
*   **Platform-Optimized Content**: Uses predefined `PLATFORM_SPECS` for Facebook, Instagram, Twitter, and LinkedIn to tailor post length, emoji usage, hashtags, and tone.
*   **Flexible Generation**:
    *   `generate_post()`: Generates single or multiple variations of social media posts based on a `topic`, `platform`, and `style`.
    *   `generate_from_blog()`: Repurposes blog content into social media posts.
    *   `generate_from_image_description()`: Creates captions from image descriptions.
    *   `generate_weekly_content()`: Generates a week's worth of content based on daily topics.
    *   `generate_engagement_response()`: Crafts responses to social media comments.
    *   `generate_hashtags()`: Generates relevant hashtags for a given topic.
*   **Prompt Engineering**: Constructs detailed prompts for the Gemini model, including brand context and specific instructions to achieve high-quality, relevant output.

**`AIContentWorkflow` Features:**
*   **Vault Integration**: Creates and manages the `/vault/Social_Media/AI_Generated/` (for draft reference) and `/vault/Pending_Approval/` directories.
*   **`create_ai_generated_post()`**: Generates content for specified platforms and creates individual markdown approval files (e.g., `AI_SOCIAL_FACEBOOK_*.md`) in `/vault/Pending_Approval/`. These files include the AI-generated post, metadata (platform, topic, style, `created_by: gemini_ai`), and clear instructions for human review and approval.
*   **`generate_and_schedule_week()`**: Orchestrates the generation of a full week's content and creates a single markdown approval file (`WEEKLY_SOCIAL_*.md`) per platform, containing all daily post options.

### Single Post Generation (`generate_post.py`)

This command-line script allows for quick, on-demand generation of single social media posts using Gemini AI.

**Usage:**
```bash
python generate_post.py "<topic>" <platform> [--style <style>] [--save] [--media-path <path>] [--media-url <url>]
```

**Arguments:**
*   **`<topic>`**: The subject for the social media post.
*   **`<platform>`**: `facebook`, `instagram`, `twitter`, or `linkedin`.
*   **`--style <style>`**: Content tone (`professional`, `casual`, `exciting`, `witty`). Default is `professional`.
*   **`--save`**: Saves the generated post(s) to `/vault/Pending_Approval/` for human review and approval.
*   **`--media-path <path>`**: (For Instagram) Local file path to an image to include with the post.
*   **`--media-url <url>`**: (For Instagram) Public URL to an image to include with the post.

**Workflow:**
1.  Generates a post based on the provided arguments.
2.  Prints the generated post to the console.
3.  If `--save` is used, an approval markdown file (`AI_SOCIAL_*.md`) is created in `/vault/Pending_Approval/`. If `media-path` or `media-url` are provided, this information is added to the frontmatter of the approval file for the Local Agent to use during publishing.

### Batch Content Generation (`social_platform/agents/batch_content_generator.py`)

This interactive script simplifies the creation of a full week's social media content across all platforms.

**Usage:**
```bash
python social_platform/agents/batch_content_generator.py
```

**Interactive Workflow:**
1.  **API Key Check**: Verifies `GEMINI_API_KEY` is configured.
2.  **Content Type Selection**: Prompts you to choose themes:
    *   Standard weekly themes.
    *   Product launch week themes.
    *   Custom themes (allows you to specify a topic for each day of the week).
3.  **Platform & Style Selection**: Allows selection of target platforms and content style.
4.  **Generation**: For each selected platform, it generates *two variations* for each day's topic, resulting in a large batch of content (e.g., 7 days * 4 platforms * 2 options = 56 posts).
5.  **Approval File Creation**: Instead of individual posts, it creates **one `WEEKLY_SOCIAL_*.md` approval file per platform** in `/vault/Pending_Approval/`. Each file contains all daily posts with their two options.
    *   **Human Selection**: The frontmatter of these weekly approval files includes detailed instructions on how to select preferred daily options (e.g., by adding `selected_options` to the frontmatter).
    *   **Editing**: Users can directly edit post content within the markdown file.
    *   **Scheduling**: Can add a `posting_schedule` to the frontmatter for timed releases.
6.  **Draft Reference**: Also saves a comprehensive `WEEKLY_CONTENT_*.md` draft in `/vault/Social_Media/AI_Generated/` for reference.

### Testing AI Features (`social_platform/agents/test_gemini_ai.py`)

This script provides a comprehensive test suite for the Gemini AI integration, serving as both a verification tool and a practical demonstration of its capabilities.

**Usage:**
```bash
python social_platform/agents/test_gemini_ai.py
```

**Key Tests & Functionality Demonstrated:**
*   **Setup Check**: Verifies `GEMINI_API_KEY` configuration and `google-generativeai` SDK installation.
*   **Single Post Generation**: Generates and displays example posts for different platforms and styles.
*   **Hashtag Generation**: Demonstrates the ability to create relevant hashtags.
*   **Engagement Response**: Shows how AI can generate replies to social media comments.
*   **Workflow Integration**: Confirms that AI-generated content can be successfully saved as approval requests in `/vault/Pending_Approval/`.
*   **Blog Repurposing**: Illustrates the transformation of blog excerpts into social posts.
*   **Clear Reporting**: Provides detailed console output for each test, indicating success or failure, and showing examples of AI-generated content.

---

## 9. Social Media Integrations

The Platinum Tier supports automated posting to various social media platforms, with specific implementation details for each.

### Instagram Posting (with Images via ngrok)

Instagram posting is handled by the Local Agent and supports both public image URLs and local image files.

**Workflow:**
1.  **Approval Request**: An `AI_SOCIAL_*.md` approval file (generated by Cloud Agent or `generate_post.py --save`) contains the post caption and either `media_url` or `media_path` in its frontmatter.
2.  **Local Agent Execution**: Upon approval, the Local Agent executes the post:
    *   **Method 1: URL Posting (Graph API)**: If `media_url` is provided, the Local Agent directly uses the Instagram Graph API to create a media container and then publish the post.
    *   **Method 2: Local File Upload (ngrok Temporary Server)**: If `media_path` points to a local image file:
        *   The Local Agent dynamically starts a temporary local HTTP server.
        *   It uses `pyngrok` to create a secure, temporary public URL tunnel to this local server, exposing the image.
        *   The Instagram Graph API fetches the image from this temporary ngrok URL.
        *   After posting, the local server and ngrok tunnel are automatically shut down. **No permanent hosting of your local files is involved.**

**Prerequisites & Configuration:**
*   **Instagram Graph API Credentials**: `INSTAGRAM_ACCESS_TOKEN` and `INSTAGRAM_ACCOUNT_ID` in your Local machine's `.env`. (Requires Facebook Page connection).
*   **`pyngrok`**: Install `pip install pyngrok`.
*   **ngrok Auth Token**: `NGROK_AUTH_TOKEN` in your Local machine's `.env`. Obtainable from [https://dashboard.ngrok.com/get-started/your-authtoken](https://dashboard.ngrok.com/get-started/your-authtoken).
*   **Supported Formats**: JPEG (`.jpg`, `.jpeg`), PNG (`.png`).
*   **Image Requirements**: Min 320x320, Recommended 1080x1080 (square), Max 4096x4096, under 8MB.

### LinkedIn Posting (via Playwright)

LinkedIn posting utilizes Playwright, a browser automation library, for robust interaction with the LinkedIn web interface.

**Workflow:**
1.  **Approval Request**: An `AI_SOCIAL_*.md` approval file contains the LinkedIn post content in its frontmatter.
2.  **Local Agent Execution**: Upon approval, the Local Agent:
    *   Launches a headless (or headful for debugging) browser instance (Chromium/Edge/Chrome).
    *   **Persistent Session**: Maintains a persistent browser session stored in `/vault/linkedin_session/`. This means you log in manually once, and the session remains active for subsequent posts.
    *   Navigates to LinkedIn.com, checks login status. If not logged in, it prompts for manual login.
    *   Locates and clicks the "Start a post" button.
    *   Types the content into the post editor.
    *   Locates and clicks the "Post" button.
    *   Closes the browser.

**Prerequisites & Configuration:**
*   **Playwright**: Install `pip install playwright`. You may also need to install browser binaries: `playwright install`.
*   **Persistent Session**: The `/vault/linkedin_session/` directory (which is excluded by `.gitignore`) must be maintained.
*   **Manual Login**: The first time (or after session expiry/corruption), you may need to manually log in within the launched browser window.

### Facebook & Twitter Posting

These platforms utilize direct API integrations.

**Configuration:**
*   **Facebook**: `FACEBOOK_ACCESS_TOKEN` and `FACEBOOK_PAGE_ID` in the relevant `.env` file (Cloud VM for drafting, Local for execution/publishing).
*   **Twitter**: `TWITTER_BEARER_TOKEN` (for API v2) or older API keys/secrets in `.env`.

---

## 10. Monitoring, Logging & Backup

Maintaining the health, operational visibility, and data integrity of your Platinum AI Employee is critical.

### Health Monitoring Script (`scripts/health-monitor.sh`)

This script (embedded in `cloud/setup-cloud-vm.sh` for Cloud VM deployments) performs periodic checks to ensure the system is operating correctly.

**Functionality:**
*   **Health Endpoint Check**: Queries the `/health` endpoint (served by Nginx on the Cloud VM) to check the overall service status.
*   **Container Status**: Lists Docker container status (`docker ps`).
*   **Resource Usage**: Reports disk usage (`df -h`) and memory usage (`free -h`).
*   **Alerting**: If issues are detected (e.g., health endpoint error), it can send an email alert to the address configured in `ALERT_EMAIL` in the `.env` file.
*   **Cron Job**: This script is typically configured to run as a cron job (e.g., every 5 minutes) on the Cloud VM.

**Configuration:**
*   `ALERT_EMAIL`: Email address for alerts in the Cloud VM's `.env`.
*   `HEALTH_CHECK_INTERVAL`: Frequency of checks in `.env`.

### Odoo Database Backup (`scripts/backup-odoo.sh`)

This script (embedded in `cloud/setup-cloud-vm.sh` for Cloud VM deployments) automates regular backups of your Odoo database.

**Functionality:**
*   **Database Backup**: Uses `docker exec odoo-db pg_dump` to create a `*.sql` dump of the Odoo PostgreSQL database and then `gzip`s it.
*   **Filestore Backup**: Backs up the Odoo filestore (attachments, documents) by packaging the `odoo-data` Docker volume.
*   **Cleanup**: Automatically deletes backups older than 30 days.
*   **Cron Job**: Configured to run daily (e.g., 2 AM) via cron on the Cloud VM.

**Configuration:**
*   This script expects the Odoo and PostgreSQL Docker containers to be running.
*   Backup files are stored in `/opt/ai-employee/odoo/backups` on the Cloud VM.

### Log Rotation

`logrotate` is configured on the Cloud VM (via `cloud-init.yaml` or `setup-cloud-vm.sh`) to manage the size and retention of log files generated by the AI Employee services.

*   **Configuration**: Log files in `/opt/ai-employee/logs/` are rotated daily, compressed, and retained for 30 days.

---

## 11. Testing & Demo Workflow

Ensuring the Platinum Tier operates as expected is crucial. The project includes comprehensive testing tools and a full demo workflow.

### Comprehensive Test Suite (`scripts/test-platinum.py`)

This script provides a robust test suite for the core components and integrations of the Platinum Tier. It's designed to verify the functionality and interaction of agents, vault sync, and security rules.

**Usage:**
```bash
python scripts/test-platinum.py
```

**Key Tests:**
*   **Vault Sync**: Verifies Git repository initialization, `.gitignore` setup within the vault, and basic sync operations.
*   **Claim Manager**: Tests the "claim-by-move" coordination rule, ensuring exclusive task processing.
*   **Cloud Agent**: Checks basic initialization and its ability to process emails and create approval requests with correct metadata.
*   **Local Agent**: Verifies initialization and its capability to detect pending approvals.
*   **Security Rules**: **Crucially**, this test verifies:
    *   The project's root `.gitignore` correctly excludes `.env`, `*.session`, `secrets/`, and `tokens/`.
    *   The `.env.example` file clearly separates Cloud and Local secrets.
*   **Demo Workflow**: Executes the full `platinum-demo.py` script and asserts that all steps of the Cloud-Local task handover are completed successfully.

### Platinum Demo (`scripts/platinum-demo.py`)

This script offers a clear, step-by-step demonstration of the end-to-end Platinum Tier workflow, illustrating the key concepts of work-zone specialization and the vault-based communication. It uses simulated interactions to show a task (email reply) moving from Cloud initiation to Local execution.

**Usage:**
```bash
python scripts/platinum-demo.py
```

**Demonstrated Workflow (9 steps):**
1.  **Email arrives**: Simulated in Cloud's `Needs_Action`.
2.  **Cloud Gmail Watcher detects**: Cloud Agent role.
3.  **Cloud Agent drafts reply**: Generates content.
4.  **Cloud creates approval request**: File in `Pending_Approval`.
5.  **Cloud syncs to vault**: Git push.
6.  **Local receives sync**: Git pull.
7.  **User approves**: Moves file to `Approved`.
8.  **Local executes send**: Simulated execution (actual Gmail API call by Local Agent).
9.  **Task completed and logged**: File moved to `Done/local/`, log entry created.

This demo runs in a temporary directory (`./demo_vault`) to avoid affecting your main project. It provides verbose console output, explaining each step and confirming the successful handoff between agents.

---

## 12. Troubleshooting & Maintenance

This section provides guidance for common issues, dependency problems, and maintenance tasks.

### General Issues

*   **Vault Sync Conflicts**: If `git pull` fails in the vault, you might need to manually resolve conflicts or use `git pull --rebase`. For unmanageable conflicts, `VaultSync.force_sync()` (or `git reset --hard origin/main`) can reset the local vault to match the remote.
*   **Agent Not Starting**: Check agent logs. For Dockerized agents, use `docker-compose logs <service_name>`. For Systemd-managed Cloud Agents, use `systemctl status cloud-agent`.
*   **Approvals Not Syncing**: Verify that your `.gitignore` files (root and vault) are not inadvertently excluding `/Pending_Approval/` or other crucial vault folders. Ensure `VaultSync` is running and configured correctly.
*   **Cloud VM Unreachable**: Check your SSH access, firewall rules on the VM, and ensure the VM is running. Restarting the VM might help.
*   **Permissions Issues**: Ensure the `ai-employee` user (or your executing user) has proper read/write permissions for the project directory, especially the `vault/` folder.

### Python & Dependency Issues

*   **`ModuleNotFoundError`**:
    *   `google.generativeai`: `pip install google-generativeai`
    *   `google-auth`, `google-auth-oauthlib`, `google-api-python-client`: `pip install google-auth google-auth-oauthlib google-api-python-client`
    *   `pyngrok`: `pip install pyngrok`
    *   `playwright`: `pip install playwright` and `playwright install` (to install browser binaries).
*   **`dotenv` not loading**: Ensure your `.env` file is in the correct location (project root for agents, potentially `/opt/ai-employee/.env` on Cloud VM) and `load_dotenv()` is called.

### Credentials & API Issues

*   **`credentials.json` not found**: Ensure `credentials.json` (Google API client secrets) is in your project root. If you were migrating from Gold tier, ensure `scripts/copy_credentials.bat` was run.
*   **`Gmail API not initialized`**:
    *   Verify `credentials.json` exists.
    *   Delete `gmail_token.json` if it's corrupted or expired, then re-run `scripts/authenticate_gmail.bat`.
    *   Ensure the Local Agent was run once to trigger the OAuth flow if `gmail_token.json` is missing.
*   **`GEMINI_API_KEY` not found/configured**: Get your key from [https://aistudio.google.com/app/apikey](https://aistudio.google.com/app/apikey) and add `GEMINI_API_KEY=your_key_here` to your `.env` file.
*   **Social Media API Errors (e.g., `403 Forbidden`)**:
    *   Verify access tokens (`FACEBOOK_ACCESS_TOKEN`, `INSTAGRAM_ACCESS_TOKEN`, `TWITTER_BEARER_TOKEN`) are correct and have necessary permissions.
    *   Ensure Instagram access tokens are linked to a Facebook Page.
    *   For LinkedIn with Playwright, ensure you are logged in within the persistent browser session.
*   **`NGROK_AUTH_TOKEN` not configured**: Obtain your token from [https://dashboard.ngrok.com/get-started/your-authtoken](https://dashboard.ngrok.com/get-started/your-authtoken) and add it to your Local `.env`.

### Odoo Troubleshooting

*   **"Odoo not connected" / "Authentication failed"**:
    *   Verify the Odoo instance is running (e.g., `docker ps | grep odoo`).
    *   Check `ODOO_URL`, `ODOO_DB`, `ODOO_USERNAME`, `ODOO_PASSWORD` in your `.env`.
    *   Test Odoo accessibility directly (e.g., open `http://localhost:8069` in a browser).
    *   Ensure the Odoo user has API access rights.
*   **"Invoice creation failed"**: Verify customer names exist in Odoo or the agent's logic for creating new partners is working. Check Odoo logs for specific errors.

### AI Content Troubleshooting

*   **Post content issues (encoding, quality)**: Edit the approval file directly. Adjust the `style` parameter or provide a more specific `topic` for generation.
*   **"File not found" (for images)**: Ensure local image paths are absolute and the file exists.
*   **"Unsupported image format"**: Instagram requires `.jpg` or `.png`. Convert if necessary.
*   **Rate Limit Exceeded (Gemini)**: Implement delays between calls or use batch generation strategies if encountering this frequently.

---

This implementation guide provides the detailed knowledge required to set up, operate, and troubleshoot your Platinum Tier AI Employee. For day-to-day operations, refer to the `README.md` and `QUICK_START.md` for concise instructions.
