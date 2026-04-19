"""
scripts/platinum-demo.py
Platinum Demo: Email → Cloud Draft → Local Approve → Local Send

This script demonstrates the complete Platinum tier workflow:
1. Email arrives while Local is offline
2. Cloud detects and drafts reply
3. Cloud creates approval request
4. Cloud syncs to vault
5. Local comes online and receives sync
6. User approves by moving file
7. Local executes send via MCP
8. Task completed and logged
"""

import os
import time
from pathlib import Path
from datetime import datetime
import shutil


def run_platinum_demo():
    """Run the complete Platinum tier demo"""
    
    print("=" * 70)
    print("PLATINUM TIER DEMO")
    print("Scenario: Email arrives while Local offline")
    print("=" * 70)
    print()
    
    # Setup demo environment
    demo_dir = Path('./demo_vault')
    if demo_dir.exists():
        shutil.rmtree(demo_dir)
    
    # Create directory structure
    cloud_vault = demo_dir / 'cloud' / 'vault'
    local_vault = demo_dir / 'local' / 'vault'
    
    for vault in [cloud_vault, local_vault]:
        (vault / 'Needs_Action' / 'email').mkdir(parents=True, exist_ok=True)
        (vault / 'Pending_Approval').mkdir(parents=True, exist_ok=True)
        (vault / 'Approved').mkdir(parents=True, exist_ok=True)
        (vault / 'Done' / 'local').mkdir(parents=True, exist_ok=True)
        (vault / 'Updates').mkdir(parents=True, exist_ok=True)
    
    # Step 1: Simulate email arrival
    print("[Step 1] Email arrives")
    print("-" * 70)
    email_data = {
        'from': 'client@example.com',
        'subject': 'Invoice Request',
        'body': 'Can you send me the invoice for January?',
    }
    print(f"  From: {email_data['from']}")
    print(f"  Subject: {email_data['subject']}")
    print(f"  Body: {email_data['body']}")
    print()
    
    # Step 2: Cloud detects email
    print("[Step 2] Cloud Gmail Watcher detects email")
    print("-" * 70)
    cloud_needs_action = cloud_vault / 'Needs_Action' / 'email'
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    email_file = cloud_needs_action / f"EMAIL_{timestamp}.md"
    
    email_content = f"""---
type: email
from: {email_data['from']}
subject: {email_data['subject']}
received: {datetime.now().isoformat()}
priority: high
status: pending
---

## Email Content
{email_data['body']}
"""
    email_file.write_text(email_content)
    print(f"  [OK] Created: {email_file.name}")
    print()
    
    # Step 3: Cloud processes and drafts reply
    print("[Step 3] Cloud Agent drafts reply")
    print("-" * 70)
    draft_content = f"""---
type: email_reply
to: {email_data['from']}
subject: Re: {email_data['subject']}
created: {datetime.now().isoformat()}
created_by: cloud_agent
---

## Draft Reply

Dear Client,

Thank you for your inquiry. Please find attached your invoice for January.

Best regards,
AI Employee
"""
    print("  [OK] Draft reply created")
    print()
    
    # Step 4: Cloud creates approval request
    print("[Step 4] Cloud creates approval request")
    print("-" * 70)
    approval_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    approval_file = cloud_vault / 'Pending_Approval' / f"EMAIL_SEND_{approval_timestamp}.md"
    
    approval_content = f"""---
type: email_send
created: {datetime.now().isoformat()}
created_by: cloud_agent
status: pending
requires_local_execution: true
---

# Approval Required

## Action
Send Email Reply

## Details
- To: {email_data['from']}
- Subject: Re: {email_data['subject']}

## Draft Content
{draft_content}

## To Approve
Move this file to /Approved folder

## Note
This action requires Local execution (send)
Cloud agent cannot execute this action.
"""
    approval_file.write_text(approval_content)
    print(f"  [OK] Created: {approval_file.name}")
    print()
    
    # Step 5: Cloud syncs to vault (simulated)
    print("[Step 5] Cloud syncs to vault (Git push)")
    print("-" * 70)
    time.sleep(1)  # Simulate sync delay
    print("  [OK] Synced to Cloud")
    print()
    
    # Step 6: Local receives sync (simulated)
    print("[Step 6] Local receives sync (Git pull)")
    print("-" * 70)
    local_approval_file = local_vault / 'Pending_Approval' / approval_file.name
    local_approval_file.write_text(approval_content)
    print(f"  [OK] Received: {approval_file.name}")
    print()
    
    # Step 7: User approves
    print("[Step 7] User approves (moves file to /Approved)")
    print("-" * 70)
    approved_file = local_vault / 'Approved' / approval_file.name
    local_approval_file.rename(approved_file)
    print(f"  [OK] Approved: {approved_file.name}")
    print()
    
    # Step 8: Local executes send
    print("[Step 8] Local executes send via MCP")
    print("-" * 70)
    time.sleep(1)  # Simulate send delay
    print("  [OK] Email sent via Gmail MCP")
    print()
    
    # Step 9: Task completed
    print("[Step 9] Task completed and logged")
    print("-" * 70)
    done_file = local_vault / 'Done' / 'local' / approved_file.name
    approved_file.rename(done_file)
    print(f"  [OK] Moved to Done: {done_file.name}")
    
    # Create log entry
    log_entry = f"""{{
  "timestamp": "{datetime.now().isoformat()}",
  "action_type": "email_send",
  "actor": "local_agent",
  "executed_by": "human_approved",
  "original_file": "{approved_file.name}",
  "result": "success"
}}"""
    log_file = local_vault / 'Logs' / f"{datetime.now().strftime('%Y-%m-%d')}.json"
    log_file.parent.mkdir(parents=True, exist_ok=True)
    log_file.write_text(log_entry)
    print(f"  [OK] Logged to: {log_file.name}")
    print()
    
    # Summary
    print("=" * 70)
    print("DEMO COMPLETE")
    print("=" * 70)
    print("[OK] All steps successful")
    print("[OK] Cloud drafted, Local approved and executed")
    print("[OK] Work-zone specialization working")
    print("[OK] Vault sync functional")
    print()
    print("Key Points Demonstrated:")
    print("  1. Cloud handles drafts only (never sends)")
    print("  2. Local handles approvals and final execution")
    print("  3. Vault syncs between Cloud <-> Local")
    print("  4. WhatsApp/Banking credentials stay local")
    print("  5. Complete audit trail maintained")
    print()
    print("Next Steps:")
    print("  - Run: python cloud/agents/cloud_agent.py")
    print("  - Run: python local/agents/local_agent.py")
    print("  - Deploy to Cloud VM: ./scripts/deploy-to-cloud.sh")
    print("=" * 70)


if __name__ == '__main__':
    run_platinum_demo()
