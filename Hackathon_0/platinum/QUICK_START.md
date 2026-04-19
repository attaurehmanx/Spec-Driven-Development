# Quick Start: Platinum Tier in 5 Minutes

**Prerequisites**: Gold tier complete, Cloud VM account, Git repository

## Step 1: Clone & Setup (1 minute)

```bash
# Clone repository
git clone <your-repo-url> platinum
cd platinum

# Install dependencies
pip install -r requirements.txt
```

## Step 2: Deploy Cloud VM (2 minutes)

```bash
# Deploy to Oracle Cloud (or AWS)
./scripts/deploy-to-cloud.sh

# Follow prompts:
# - Select cloud provider (Oracle/AWS)
# - Enter SSH key path
# - Enter domain (optional for HTTPS)

# Wait 3-5 minutes for deployment to complete
```

## Step 3: Configure Local (1 minute)

```bash
# Copy environment template
cp .env.example .env

# Edit .env with LOCAL credentials only:
nano .env

# Required for Local:
WHATSAPP_SESSION_PATH=/path/to/session
BANKING_API_KEY=your_local_banking_key
PAYMENT_TOKEN=your_payment_token

# DO NOT add Cloud credentials here
```

## Step 4: Setup Vault Sync (30 seconds)

```bash
# Initialize Git sync
./scripts/setup-vault-sync.sh

# Enter your Git repository URL
# This syncs Cloud ↔ Local vault
```

## Step 5: Start Local Agent (30 seconds)

```bash
# Start Local orchestrator
python local/orchestrator.py

# You should see:
# [✓] Local agent started
# [✓] Vault sync active
# [✓] Waiting for approvals...
```

## Test the Platinum Demo (2 minutes)

```bash
# Run automated test
python scripts/test-platinum-demo.py

# Expected output:
# [✓] All systems operational
# [✓] Cloud-Local sync working
# [✓] Approval workflow functional
```

## You're Ready!

Your Platinum AI Employee is now:
- ✅ Running 24/7 on Cloud VM
- ✅ Syncing vault between Cloud ↔ Local
- ✅ Cloud handling drafts
- ✅ Local handling approvals + final actions

## Next Steps

1. **Monitor Dashboard**: Open `vault/Dashboard.md` in Obsidian
2. **Check Cloud Health**: `curl https://your-cloud-vm.com/health`
3. **Review Pending Approvals**: Check `vault/Pending_Approval/` folder
4. **Read Full Guide**: See `IMPLEMENTATION_GUIDE.md` for details

## Quick Commands

```bash
# Check Cloud status
ssh user@cloud-vm "systemctl status cloud-agent"

# Force vault sync
./scripts/force-sync.sh

# View logs
tail -f logs/local-agent.log

# Stop Local agent
Ctrl+C
```

## Common Issues

| Problem | Quick Fix |
|---------|-----------|
| Sync failed | `git pull --rebase` in vault folder |
| Cloud unreachable | Check SSH access, restart VM |
| Approvals not showing | Verify folder permissions |

---

**Need Help?** See `IMPLEMENTATION_GUIDE.md` for detailed troubleshooting.
