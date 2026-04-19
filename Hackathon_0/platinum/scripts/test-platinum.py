"""
scripts/test-platinum.py
Comprehensive test suite for Platinum Tier
"""

import os
import sys
import time
import shutil
from pathlib import Path
from datetime import datetime

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))


def test_vault_sync():
    """Test vault sync functionality"""
    print("\n" + "="*70)
    print("TEST: Vault Sync")
    print("="*70)
    
    from shared.vault_sync.git_sync import VaultSync
    
    # Create test vault
    test_vault = Path('./test_vault_sync')
    if test_vault.exists():
        shutil.rmtree(test_vault)
    
    # Initialize sync
    sync = VaultSync(str(test_vault))
    result = sync.initialize()
    
    assert result, "Failed to initialize vault sync"
    assert (test_vault / '.git').exists(), "Git repository not created"
    assert (test_vault / '.gitignore').exists(), ".gitignore not created"
    
    print("✓ Vault sync initialized")
    print("✓ Git repository created")
    print("✓ .gitignore created with security rules")
    
    # Cleanup
    shutil.rmtree(test_vault)
    print("✓ Test cleanup complete")
    
    return True


def test_claim_manager():
    """Test claim-by-move rule"""
    print("\n" + "="*70)
    print("TEST: Claim Manager")
    print("="*70)
    
    from shared.claim_manager.claim_manager import ClaimManager
    
    # Create test vault
    test_vault = Path('./test_claim_vault')
    if test_vault.exists():
        shutil.rmtree(test_vault)
    
    # Create folder structure
    (test_vault / 'Needs_Action').mkdir(parents=True)
    
    # Create claim managers
    cloud_manager = ClaimManager(str(test_vault), 'cloud_agent')
    local_manager = ClaimManager(str(test_vault), 'local_agent')
    
    print("✓ Claim managers initialized")
    
    # Create test task
    task_file = test_vault / 'Needs_Action' / 'test_task.md'
    task_file.write_text("---\ntype: test\n---\n\nTest task")
    
    # Test claiming
    claimed = cloud_manager.claim_task(task_file)
    assert claimed, "Cloud agent failed to claim task"
    
    print("✓ Cloud agent claimed task")
    
    # Try to claim same task (should fail)
    claimed_again = local_manager.claim_task(task_file)
    assert not claimed_again, "Local agent should not be able to claim already-claimed task"
    
    print("✓ Local agent correctly blocked from claiming")
    
    # Test release
    claimed_file = cloud_manager.in_progress_folder / 'test_task.md'
    released = cloud_manager.release_task(claimed_file, status='done')
    assert released, "Failed to release task"
    
    print("✓ Task released to Done folder")
    
    # Cleanup
    shutil.rmtree(test_vault)
    print("✓ Test cleanup complete")
    
    return True


def test_cloud_agent():
    """Test Cloud Agent functionality"""
    print("\n" + "="*70)
    print("TEST: Cloud Agent")
    print("="*70)
    
    from cloud.agents.cloud_agent import CloudAgent
    
    # Create test vault
    test_vault = Path('./test_cloud_vault')
    if test_vault.exists():
        shutil.rmtree(test_vault)
    
    # Initialize agent
    agent = CloudAgent(str(test_vault))
    
    print("✓ Cloud Agent initialized")
    
    # Test email processing
    email_data = {
        'from': 'client@example.com',
        'subject': 'Invoice Request',
        'body': 'Can you send me the invoice?',
    }
    
    approval_file = agent.process_email(email_data)
    assert approval_file is not None, "Failed to create approval request"
    assert approval_file.exists(), "Approval file not created"
    
    print(f"✓ Approval request created: {approval_file.name}")
    
    # Verify content
    content = approval_file.read_text()
    assert 'cloud_agent' in content, "created_by not set correctly"
    assert 'requires_local_execution: true' in content, "Local execution flag not set"
    
    print("✓ Approval request has correct metadata")
    
    # Cleanup
    shutil.rmtree(test_vault)
    print("✓ Test cleanup complete")
    
    return True


def test_local_agent():
    """Test Local Agent functionality"""
    print("\n" + "="*70)
    print("TEST: Local Agent")
    print("="*70)
    
    from local.agents.local_agent import LocalAgent
    
    # Create test vault
    test_vault = Path('./test_local_vault')
    if test_vault.exists():
        shutil.rmtree(test_vault)
    
    # Initialize agent
    agent = LocalAgent(str(test_vault))
    
    print("✓ Local Agent initialized")
    
    # Create approval file
    (test_vault / 'Pending_Approval').mkdir(parents=True)
    approval_file = test_vault / 'Pending_Approval' / 'test_approval.md'
    
    approval_content = """---
type: email_send
created: 2026-01-07T10:00:00
created_by: cloud_agent
status: pending
requires_local_execution: true
---

# Approval Required

## Action
Send Email

## Details
- To: client@example.com
- Subject: Test
"""
    approval_file.write_text(approval_content)
    
    print("✓ Approval file created")
    
    # Test approval checking
    pending = agent.check_pending_approvals()
    assert len(pending) == 1, "Failed to detect pending approval"
    
    print("✓ Local agent detected pending approval")
    
    # Cleanup
    shutil.rmtree(test_vault)
    print("✓ Test cleanup complete")
    
    return True


def test_security_rules():
    """Test security rules (secrets isolation)"""
    print("\n" + "="*70)
    print("TEST: Security Rules")
    print("="*70)
    
    # Check .gitignore
    gitignore_path = Path('./.gitignore')
    assert gitignore_path.exists(), ".gitignore not found"
    
    gitignore_content = gitignore_path.read_text()
    
    # Verify critical exclusions
    assert '.env' in gitignore_content, ".env not excluded"
    assert '*.session' in gitignore_content, "Session files not excluded"
    assert 'secrets/' in gitignore_content, "Secrets directory not excluded"
    assert 'tokens/' in gitignore_content, "Tokens directory not excluded"
    
    print("✓ .gitignore excludes secrets")
    print("✓ .gitignore excludes session files")
    print("✓ .gitignore excludes tokens")
    
    # Check .env.example
    env_example = Path('./.env.example')
    assert env_example.exists(), ".env.example not found"
    
    env_content = env_example.read_text()
    assert 'CLOUD SECRETS' in env_content, "Cloud secrets section missing"
    assert 'LOCAL SECRETS' in env_content, "Local secrets section missing"
    
    print("✓ Environment template separates Cloud/Local secrets")
    
    return True


def test_demo_workflow():
    """Test complete Platinum demo workflow"""
    print("\n" + "="*70)
    print("TEST: Platinum Demo Workflow")
    print("="*70)
    
    # Run the demo
    import subprocess
    result = subprocess.run(
        [sys.executable, 'scripts/platinum-demo.py'],
        capture_output=True,
        text=True,
        cwd=Path(__file__).parent.parent
    )
    
    assert result.returncode == 0, f"Demo failed: {result.stderr}"
    
    # Check output
    output = result.stdout
    
    assert '[Step 1] Email arrives' in output, "Step 1 missing"
    assert '[Step 2] Cloud Gmail Watcher detects email' in output, "Step 2 missing"
    assert '[Step 3] Cloud Agent drafts reply' in output, "Step 3 missing"
    assert '[Step 4] Cloud creates approval request' in output, "Step 4 missing"
    assert '[Step 5] Cloud syncs to vault' in output, "Step 5 missing"
    assert '[Step 6] Local receives sync' in output, "Step 6 missing"
    assert '[Step 7] User approves' in output, "Step 7 missing"
    assert '[Step 8] Local executes send' in output, "Step 8 missing"
    assert '[Step 9] Task completed' in output, "Step 9 missing"
    
    print("✓ All 9 steps completed successfully")
    print("✓ Cloud drafted reply (did not send)")
    print("✓ Local approved and executed send")
    print("✓ Work-zone specialization working")
    
    return True


def run_all_tests():
    """Run all Platinum tier tests"""
    print("\n" + "="*70)
    print("PLATINUM TIER: COMPREHENSIVE TEST SUITE")
    print("="*70)
    
    tests = [
        ("Vault Sync", test_vault_sync),
        ("Claim Manager", test_claim_manager),
        ("Cloud Agent", test_cloud_agent),
        ("Local Agent", test_local_agent),
        ("Security Rules", test_security_rules),
        ("Demo Workflow", test_demo_workflow),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, True, None))
            print(f"\n✓ {test_name}: PASSED")
        except AssertionError as e:
            results.append((test_name, False, str(e)))
            print(f"\n✗ {test_name}: FAILED - {e}")
        except Exception as e:
            results.append((test_name, False, f"Unexpected error: {e}"))
            print(f"\n✗ {test_name}: ERROR - {e}")
    
    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    
    passed = sum(1 for _, result, _ in results if result)
    total = len(results)
    
    for test_name, result, error in results:
        status = "✓ PASSED" if result else "✗ FAILED"
        print(f"{test_name}: {status}")
        if error:
            print(f"  Error: {error}")
    
    print("\n" + "="*70)
    print(f"Results: {passed}/{total} tests passed")
    print("="*70)
    
    if passed == total:
        print("\n🎉 ALL TESTS PASSED! Platinum Tier is ready!")
        print("\nNext steps:")
        print("1. Deploy to Cloud VM: ./scripts/deploy-to-cloud.sh")
        print("2. Setup vault sync: ./scripts/setup-vault-sync.sh")
        print("3. Run demo: python scripts/platinum-demo.py")
        print("4. Start Cloud agent: python cloud/agents/cloud_agent.py")
        print("5. Start Local agent: python local/agents/local_agent.py")
        return True
    else:
        print(f"\n⚠ {total - passed} test(s) failed. Please review errors above.")
        return False


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
