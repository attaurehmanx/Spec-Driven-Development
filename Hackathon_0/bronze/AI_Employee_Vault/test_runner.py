"""
Simple test runner for AI Employee system
Tests basic functionality of the watcher and vault structure
"""
import sys
import os
from pathlib import Path
from datetime import datetime

# Set UTF-8 encoding for Windows console
if sys.platform == 'win32':
    os.system('chcp 65001 > nul')
    sys.stdout.reconfigure(encoding='utf-8')

def test_vault_structure():
    """Test that all required folders exist"""
    print("Testing vault structure...")

    required_folders = [
        'Inbox',
        'Needs_Action',
        'Done',
        'Plans',
        'Logs',
        'Pending_Approval',
        'Approved',
        'Rejected',
        'Briefings'
    ]

    vault_path = Path('vault')
    all_exist = True

    for folder in required_folders:
        folder_path = vault_path / folder
        if folder_path.exists():
            print(f"  ✓ {folder}/ exists")
        else:
            print(f"  ✗ {folder}/ missing")
            all_exist = False

    return all_exist

def test_core_files():
    """Test that core markdown files exist"""
    print("\nTesting core files...")

    required_files = [
        'Dashboard.md',
        'Company_Handbook.md'
    ]

    vault_path = Path('vault')
    all_exist = True

    for file in required_files:
        file_path = vault_path / file
        if file_path.exists():
            print(f"  ✓ {file} exists")
        else:
            print(f"  ✗ {file} missing")
            all_exist = False

    return all_exist

def test_python_files():
    """Test that Python watcher files exist"""
    print("\nTesting Python files...")

    required_files = [
        'base_watcher.py',
        'filesystem_watcher.py',
        'requirements.txt'
    ]

    vault_path = Path('.')
    all_exist = True

    for file in required_files:
        file_path = vault_path / file
        if file_path.exists():
            print(f"  ✓ {file} exists")
        else:
            print(f"  ✗ {file} missing")
            all_exist = False

    return all_exist

def test_agent_skills():
    """Test that Agent Skills are properly configured"""
    print("\nTesting Agent Skills...")

    skills_path = Path('../.claude/skills')

    if not skills_path.exists():
        print(f"  ✗ Skills directory missing: {skills_path}")
        return False

    required_skills = [
        'vault-manager/SKILL.md',
        'daily-briefing/SKILL.md'
    ]

    all_exist = True

    for skill in required_skills:
        skill_path = skills_path / skill
        if skill_path.exists():
            print(f"  ✓ {skill} exists")
        else:
            print(f"  ✗ {skill} missing")
            all_exist = False

    return all_exist

def test_dependencies():
    """Test that Python dependencies are installed"""
    print("\nTesting Python dependencies...")

    try:
        import watchdog
        print("  ✓ watchdog installed")
        return True
    except ImportError:
        print("  ✗ watchdog not installed")
        print("    Run: pip install -r requirements.txt")
        return False

def create_test_file():
    """Create a test file in Needs_Action"""
    print("\nCreating test file...")

    test_file = Path('vault/Needs_Action/TEST_System_Check.md')

    content = f"""---
type: test_task
created: {datetime.now().isoformat()}
status: pending
priority: low
---

## System Test Task

This is an automated test file created by the test runner.

**Purpose**: Verify that the AI Employee can read and process files from Needs_Action folder.

## Suggested Actions
- [ ] Read this file
- [ ] Acknowledge the test
- [ ] Update Dashboard.md
- [ ] Move this file to /Done

## Expected Outcome
- File successfully processed
- Dashboard updated
- File moved to Done folder

---
*Created by test_runner.py*
"""

    test_file.write_text(content, encoding='utf-8')
    print(f"  ✓ Test file created: {test_file}")
    return True

def run_all_tests():
    """Run all tests and report results"""
    print("="*60)
    print("AI Employee System - Test Runner")
    print("="*60)
    print()

    results = []

    results.append(("Vault Structure", test_vault_structure()))
    results.append(("Core Files", test_core_files()))
    results.append(("Python Files", test_python_files()))
    results.append(("Agent Skills", test_agent_skills()))
    results.append(("Dependencies", test_dependencies()))
    results.append(("Test File Creation", create_test_file()))

    print()
    print("="*60)
    print("Test Summary")
    print("="*60)

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for test_name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status} - {test_name}")

    print()
    print(f"Results: {passed}/{total} tests passed")

    if passed == total:
        print()
        print("🎉 All tests passed! Your Bronze tier implementation is ready.")
        print()
        print("Next steps:")
        print("1. Start the watcher: python filesystem_watcher.py")
        print("2. Start Claude Code: claude")
        print("3. Process the test task: /vault-manager")
        return 0
    else:
        print()
        print("⚠ Some tests failed. Please fix the issues above.")
        return 1

if __name__ == '__main__':
    sys.exit(run_all_tests())
