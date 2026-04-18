"""
Test Runner for AI Employee - Silver Tier
Validates that all components are properly set up
"""
import sys
from pathlib import Path
import json

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    END = '\033[0m'

def print_header():
    print("=" * 60)
    print("AI Employee System - Test Runner (Silver Tier)")
    print("=" * 60)
    print()

def test_vault_structure():
    """Test that vault directory structure exists"""
    print("Testing vault structure...")

    vault = Path('vault')
    required_folders = [
        'Inbox',
        'Needs_Action',
        'Done',
        'Plans',
        'Logs',
        'Pending_Approval',
        'Approved',
        'Rejected',
        'Briefings',
        'LinkedIn_Drafts'
    ]

    missing = []
    for folder in required_folders:
        if not (vault / folder).exists():
            missing.append(folder)

    if missing:
        print(f"{Colors.RED}[FAIL]{Colors.END} - Missing folders: {', '.join(missing)}")
        return False
    else:
        print(f"{Colors.GREEN}[PASS]{Colors.END} - Vault structure ({len(required_folders)}/{len(required_folders)} folders)")
        return True

def test_core_files():
    """Test that core markdown files exist"""
    print("Testing core files...")

    vault = Path('vault')
    required_files = [
        'Dashboard.md',
        'Company_Handbook.md'
    ]

    missing = []
    for file in required_files:
        if not (vault / file).exists():
            missing.append(file)

    if missing:
        print(f"{Colors.RED}[FAIL]{Colors.END} - Missing files: {', '.join(missing)}")
        return False
    else:
        print(f"{Colors.GREEN}[PASS]{Colors.END} - Core files ({len(required_files)}/{len(required_files)} files)")
        return True

def test_python_files():
    """Test that Python watcher scripts exist"""
    print("Testing Python files...")

    required_scripts = [
        'base_watcher.py',
        'filesystem_watcher.py',
        'gmail_watcher.py',
        'whatsapp_watcher.py',
        'linkedin_watcher.py',
        'linkedin_poster.py',
        'orchestrator.py'
    ]

    missing = []
    for script in required_scripts:
        if not Path(script).exists():
            missing.append(script)

    if missing:
        print(f"{Colors.RED}[FAIL]{Colors.END} - Missing scripts: {', '.join(missing)}")
        return False
    else:
        print(f"{Colors.GREEN}[PASS]{Colors.END} - Python files ({len(required_scripts)}/{len(required_scripts)} scripts)")
        return True

def test_agent_skills():
    """Test that Agent Skills are configured"""
    print("Testing Agent Skills...")

    skills_dir = Path('../.claude/skills')
    required_skills = [
        'vault-manager',
        'daily-briefing',
        'linkedin-poster',
        'email-sender'
    ]

    missing = []
    for skill in required_skills:
        skill_file = skills_dir / skill / 'SKILL.md'
        if not skill_file.exists():
            missing.append(skill)

    if missing:
        print(f"{Colors.YELLOW}[WARN]{Colors.END} - Missing skills: {', '.join(missing)}")
        print(f"  Skills should be in: {skills_dir.absolute()}")
        return True  # Not critical for basic functionality
    else:
        print(f"{Colors.GREEN}[PASS]{Colors.END} - Agent Skills ({len(required_skills)}/{len(required_skills)} skills)")
        return True

def test_dependencies():
    """Test that required Python packages are installed"""
    print("Testing dependencies...")

    required_packages = [
        'watchdog'
    ]

    missing = []
    for package in required_packages:
        try:
            __import__(package)
        except ImportError:
            missing.append(package)

    if missing:
        print(f"{Colors.RED}[FAIL]{Colors.END} - Missing packages: {', '.join(missing)}")
        print(f"  Run: pip install -r requirements.txt")
        return False
    else:
        print(f"{Colors.GREEN}[PASS]{Colors.END} - Dependencies (core packages installed)")
        return True

def test_drop_folder():
    """Test that Drop_Folder exists"""
    print("Testing Drop Folder...")

    drop_folder = Path('../Drop_Folder')

    if not drop_folder.exists():
        print(f"{Colors.YELLOW}[WARN]{Colors.END} - Drop_Folder not found, creating...")
        drop_folder.mkdir(parents=True, exist_ok=True)
        print(f"{Colors.GREEN}[PASS]{Colors.END} - Drop Folder created")
        return True
    else:
        print(f"{Colors.GREEN}[PASS]{Colors.END} - Drop Folder exists")
        return True

def test_file_creation():
    """Test that we can create files in the vault"""
    print("Testing file creation...")

    test_file = Path('vault/Needs_Action/TEST_SYSTEM_CHECK.md')

    try:
        content = f"""---
type: test
created: 2026-02-20
status: pending
---

## System Test

This is a test file created by the test runner.

You can safely delete this file or process it with `/vault-manager`.
"""
        test_file.write_text(content, encoding='utf-8')

        if test_file.exists():
            print(f"{Colors.GREEN}[PASS]{Colors.END} - File creation successful")
            return True
        else:
            print(f"{Colors.RED}[FAIL]{Colors.END} - File creation failed")
            return False

    except Exception as e:
        print(f"{Colors.RED}[FAIL]{Colors.END} - Error: {e}")
        return False

def run_all_tests():
    """Run all tests and report results"""
    print_header()

    tests = [
        test_vault_structure,
        test_core_files,
        test_python_files,
        test_agent_skills,
        test_dependencies,
        test_drop_folder,
        test_file_creation
    ]

    results = []
    for test in tests:
        result = test()
        results.append(result)
        print()

    # Summary
    passed = sum(results)
    total = len(results)

    print("=" * 60)
    print(f"Results: {passed}/{total} tests passed ({passed*100//total}%)")
    print("=" * 60)
    print()

    if passed == total:
        print(f"{Colors.GREEN}SUCCESS! All tests passed! Your Silver tier implementation is ready.{Colors.END}")
        print()
        print("Next steps:")
        print("1. Start watchers: python orchestrator.py")
        print("2. Open vault in Obsidian")
        print("3. Test with Claude Code: /vault-manager")
        print()
        return 0
    else:
        print(f"{Colors.YELLOW}WARNING: Some tests failed. Please fix the issues above.{Colors.END}")
        print()
        return 1

if __name__ == '__main__':
    sys.exit(run_all_tests())
