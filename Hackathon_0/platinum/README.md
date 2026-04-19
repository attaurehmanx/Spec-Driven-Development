# Platinum Tier: Always-On Cloud + Local Executive AI Employee

**Tagline:** *24/7 Autonomous Operations with Cloud-Local Hybrid Architecture*

## Overview

The Platinum tier extends the Gold tier by adding:
- **Cloud deployment** for 24/7 always-on operation
- **Work-Zone Specialization** (Cloud handles drafts, Local handles approvals/final actions)
- **Vault Sync** between Cloud and Local environments
- **Multi-Agent coordination** via claim-by-move rules
- **Production-grade monitoring** and health checks

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         CLOUD VM (24/7)                         │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  Cloud Agent (Always-On)                                  │  │
│  │  - Email Triage + Draft Replies                           │  │
│  │  - Social Post Drafts + Scheduling                        │  │
│  │  - Odoo Integration (Draft Accounting Actions)            │  │
│  │  - Watchers: Gmail, Social Media, Odoo                    │  │
│  └───────────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  Cloud Vault Sync                                         │  │
│  │  - Git-based sync (markdown/state only)                   │  │
│  │  - NO secrets (.env, tokens, sessions never sync)         │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ Git Sync (markdown/state only)
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    LOCAL MACHINE                                │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  Local Agent (Human-in-the-Loop)                          │  │
│  │  - Review & Approve Drafts                                │  │
│  │  - WhatsApp Session (secure, local only)                  │  │
│  │  - Payments/Banking (final execution)                     │  │
│  │  - Final Send/Post Actions                                │  │
│  └───────────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  Local Vault                                              │  │
│  │  - Dashboard.md (single-writer: Local only)               │  │
│  │  - /Pending_Approval/ → /Approved/ → Execute              │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Work-Zone Specialization

| Domain | Cloud Responsibilities | Local Responsibilities |
|--------|----------------------|----------------------|
| **Email** | Triage, draft replies | Review, approve, send |
| **Social Media** | Draft posts, schedule drafts | Approve, publish |
| **Accounting (Odoo)** | Draft invoices, draft payments | Approve, post |
| **WhatsApp** | ❌ Never touches session | ✅ Full session control |
| **Payments** | ❌ Never executes | ✅ Final execution |
| **Banking** | ❌ No credentials | ✅ All credentials |

## Quick Start

### Prerequisites

1. Complete Gold tier implementation
2. Cloud VM account (Oracle Cloud Free Tier or AWS EC2)
3. Domain name for HTTPS (optional but recommended)
4. Git repository for vault sync

### Installation

```bash
# 1. Clone the Platinum repository
git clone <your-repo-url>
cd platinum

# 2. Install dependencies
pip install -r requirements.txt

# 3. Deploy to Cloud VM
./scripts/deploy-to-cloud.sh

# 4. Configure Local environment
cp .env.example .env
# Edit .env with your local credentials

# 5. Start Local agent
python scripts/local_agent.py
```

## Directory Structure

```
platinum/
├── README.md                    # This file
├── QUICK_START.md              # 5-minute setup guide
├── IMPLEMENTATION_GUIDE.md     # Full implementation guide
├── cloud/                      # Cloud VM code
│   ├── orchestrator.py         # Cloud orchestrator (24/7)
│   ├── watchers/               # Cloud watchers (Gmail, Social)
│   ├── agents/                 # Cloud agent skills
│   └── scripts/                # Cloud deployment scripts
├── local/                      # Local machine code
│   ├── orchestrator.py         # Local orchestrator
│   ├── watchers/               # Local watchers (WhatsApp)
│   ├── agents/                 # Local agent skills
│   └── scripts/                # Local setup scripts
├── shared/                     # Shared code (both Cloud + Local)
│   ├── vault_sync/             # Git-based vault sync
│   ├── claim_manager/          # Claim-by-move coordination
│   └── health_monitor/         # Health monitoring
├── vault/                      # Obsidian vault template
│   ├── Inbox/
│   ├── Needs_Action/
│   │   ├── cloud/              # Cloud-created tasks
│   │   └── local/              # Local-created tasks
│   ├── In_Progress/
│   │   ├── cloud_agent/        # Cloud-owned tasks
│   │   └── local_agent/        # Local-owned tasks
│   ├── Pending_Approval/
│   ├── Approved/
│   ├── Done/
│   │   ├── cloud/
│   │   └── local/
│   └── Updates/                # Cloud→Local sync folder
├── scripts/
│   ├── deploy-to-cloud.sh      # Cloud deployment
│   ├── setup-vault-sync.sh     # Git sync setup
│   └── health-check.sh         # Health monitoring
├── docker/
│   ├── odoo/                   # Odoo deployment
│   └── cloud-agent/            # Cloud agent container
└── requirements.txt
```

## Key Features

### 1. Cloud VM Deployment (24/7)

- **Oracle Cloud Free Tier**: 4 OCPU, 24GB RAM ARM instance (subject to availability)
- **AWS EC2**: t3.medium or higher
- Auto-deployment with Docker Compose
- HTTPS with Let's Encrypt
- Automated backups

### 2. Vault Sync (Git-Based)

```bash
# Cloud pushes updates every 5 minutes
*/5 * * * * cd /opt/ai-employee/vault && git push

# Local pulls updates every 5 minutes
*/5 * * * * cd ~/ai-employee/vault && git pull
```

**Security Rule**: `.gitignore` excludes:
- `.env` files
- `*.session` (WhatsApp sessions)
- `secrets/` directory
- `tokens/` directory

### 3. Claim-by-Move Rule

```python
# First agent to move task to their folder owns it
def claim_task(task_file, agent_name):
    target = f"In_Progress/{agent_name}/{task_file.name}"
    if target.exists():
        return False  # Already claimed
    task_file.rename(target)
    return True  # Successfully claimed
```

### 4. Health Monitoring

```bash
# Check every 60 seconds
./scripts/health-check.sh

# Alerts via email if:
# - Cloud agent down > 5 minutes
# - Vault sync failed > 30 minutes
# - Disk usage > 90%
# - Memory usage > 95%
```

## Platinum Demo Flow

**Scenario**: Email arrives while Local is offline

1. **Email arrives** → Gmail Watcher (Cloud) detects it
2. **Cloud processes** → Creates draft reply in `/Pending_Approval/email/`
3. **Cloud syncs** → Git push updates vault
4. **Local offline** → Task waits in synced vault
5. **Local comes online** → Git pull receives task
6. **User reviews** → Moves file to `/Approved/`
7. **Local executes** → Sends email via MCP
8. **Logs & completes** → Moves to `/Done/local/`, syncs back to Cloud

## Security Architecture

### Secrets Isolation

| Secret Type | Stored On Cloud? | Stored On Local? |
|-------------|-----------------|-----------------|
| Gmail API credentials | ✅ Yes | ❌ No |
| Social media tokens | ✅ Yes | ❌ No |
| Odoo admin password | ✅ Yes | ❌ No |
| WhatsApp session | ❌ **NO** | ✅ Yes |
| Banking credentials | ❌ **NO** | ✅ Yes |
| Payment tokens | ❌ **NO** | ✅ Yes |

### Vault Sync Rules

1. **Single-writer rule**: Only Local writes to `Dashboard.md`
2. **Cloud writes to**: `/Updates/` or `/Signals/`
3. **Local merges**: Cloud updates into `Dashboard.md`
4. **Never sync**: `.env`, `*.session`, `secrets/*`, `tokens/*`

## Monitoring & Alerting

### Health Endpoints

```bash
# Cloud health check
curl https://your-cloud-vm.com/health

# Response:
{
  "status": "healthy",
  "uptime": "72h 15m",
  "last_sync": "2026-01-07T10:30:00Z",
  "pending_approvals": 3,
  "disk_usage": "45%",
  "memory_usage": "62%"
}
```

### Alert Thresholds

| Metric | Warning | Critical | Action |
|--------|---------|----------|--------|
| CPU Usage | > 80% | > 95% | Auto-scale or alert |
| Memory | > 85% | > 95% | Restart agents |
| Disk | > 80% | > 90% | Clean logs |
| Sync Lag | > 10 min | > 30 min | Force sync |
| Agent Down | > 2 min | > 5 min | Auto-restart |

## Deployment Options

### Option 1: Oracle Cloud Free Tier (Recommended)

```bash
# ARM-based Ampere A1 Compute
# 4 OCPUs, 24GB RAM, 200GB storage
# Always Free (subject to availability)

./scripts/deploy-oracle-cloud.sh
```

### Option 2: AWS EC2

```bash
# t3.medium (2 vCPU, 8GB RAM)
# ~$30-40/month

./scripts/deploy-aws-ec2.sh
```

### Option 3: Self-Hosted Server

```bash
# Any machine with:
# - 8GB+ RAM
# - 4+ CPU cores
# - 50GB+ storage
# - Static IP or dynamic DNS

./scripts/deploy-self-hosted.sh
```

## Testing the Platinum Tier

### Minimum Passing Gate

The Platinum demo **must** show:

1. ✅ Email arrives while Local is offline
2. ✅ Cloud drafts reply + writes approval file
3. ✅ Local user approves when back online
4. ✅ Local executes send via MCP
5. ✅ Logs and moves task to `/Done`

### Test Script

```bash
# Run the Platinum demo test
python scripts/test-platinum-demo.py

# Expected output:
# [✓] Cloud detected email while Local offline
# [✓] Cloud created draft reply
# [✓] Cloud synced to vault
# [✓] Local received sync update
# [✓] User approved via file move
# [✓] Local executed send
# [✓] Task completed and logged
```

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| Vault sync conflicts | Run `git pull --rebase` on Local |
| Cloud agent not starting | Check `systemctl status cloud-agent` |
| Approvals not syncing | Verify `.gitignore` doesn't exclude `/Pending_Approval/` |
| WhatsApp session lost | Re-scan QR code on Local machine |

### Logs

```bash
# Cloud logs
tail -f /var/log/ai-employee/cloud-agent.log
tail -f /var/log/ai-employee/vault-sync.log

# Local logs
tail -f ~/ai-employee/logs/local-agent.log
tail -f ~/ai-employee/logs/vault-sync.log
```

## Next Steps

1. **Read QUICK_START.md** for 5-minute setup
2. **Read IMPLEMENTATION_GUIDE.md** for detailed architecture
3. **Run the Platinum demo** to verify everything works
4. **Deploy to production** with your own data

## Support

- **Documentation**: See `IMPLEMENTATION_GUIDE.md`
- **Issues**: GitHub Issues
- **Community**: Wednesday Research Meeting (Zoom, 10 PM)

---

**Built on the shoulders of Gold tier** - Complete the Gold tier requirements before attempting Platinum.

**Estimated Build Time**: 60+ hours

**Production Status**: Ready for 24/7 deployment with proper monitoring
