"""
System Test Script - Gold Tier
Tests all components of the AI Employee system
"""
import sys
from pathlib import Path
from datetime import datetime
import json

class SystemTester:
    """Test all system components"""

    def __init__(self, vault_path: str):
        self.vault_path = Path(vault_path)
        self.results = []

    def log_test(self, name: str, passed: bool, message: str = ""):
        """Log test result"""
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{status}: {name}")
        if message:
            print(f"   {message}")

        self.results.append({
            'test': name,
            'passed': passed,
            'message': message,
            'timestamp': datetime.now().isoformat()
        })

    def test_vault_structure(self):
        """Test vault folder structure"""
        print("\n" + "="*60)
        print("Testing Vault Structure")
        print("="*60)

        required_folders = [
            'Inbox', 'Needs_Action', 'In_Progress', 'Done',
            'Pending_Approval', 'Approved', 'Rejected',
            'Plans', 'Logs', 'Briefings', 'Accounting'
        ]

        for folder in required_folders:
            folder_path = self.vault_path / folder
            exists = folder_path.exists() and folder_path.is_dir()
            self.log_test(f"Folder: {folder}", exists)

    def test_core_files(self):
        """Test core markdown files"""
        print("\n" + "="*60)
        print("Testing Core Files")
        print("="*60)

        required_files = [
            'Dashboard.md',
            'Company_Handbook.md',
            'Business_Goals.md'
        ]

        for file in required_files:
            file_path = self.vault_path / file
            exists = file_path.exists() and file_path.is_file()

            if exists:
                # Check file is not empty
                content = file_path.read_text(encoding='utf-8')
                not_empty = len(content.strip()) > 0
                self.log_test(f"File: {file}", not_empty,
                            f"Size: {len(content)} bytes")
            else:
                self.log_test(f"File: {file}", False, "File not found")

    def test_watchers(self):
        """Test watcher scripts exist"""
        print("\n" + "="*60)
        print("Testing Watcher Scripts")
        print("="*60)

        watchers_path = self.vault_path.parent / 'watchers'

        required_watchers = [
            'base_watcher.py',
            'gmail_watcher.py',
            'whatsapp_watcher.py',
            'linkedin_watcher.py',
            'social_media_watcher.py',
            'odoo_watcher.py',
            'filesystem_watcher.py'
        ]

        for watcher in required_watchers:
            watcher_path = watchers_path / watcher
            exists = watcher_path.exists()
            self.log_test(f"Watcher: {watcher}", exists)

    def test_orchestrator(self):
        """Test orchestrator exists"""
        print("\n" + "="*60)
        print("Testing Orchestrator")
        print("="*60)

        orchestrator_path = self.vault_path.parent / 'orchestrator.py'
        exists = orchestrator_path.exists()
        self.log_test("Orchestrator script", exists)

    def test_skills(self):
        """Test Claude Code skills"""
        print("\n" + "="*60)
        print("Testing Agent Skills")
        print("="*60)

        skills_path = self.vault_path.parent.parent / '.claude' / 'skills'

        required_skills = [
            'vault-manager',
            'ceo-briefing',
            'social-poster'
        ]

        for skill in required_skills:
            skill_path = skills_path / skill / 'SKILL.md'
            exists = skill_path.exists()
            self.log_test(f"Skill: {skill}", exists)

    def test_drop_folder(self):
        """Test Drop_Folder exists"""
        print("\n" + "="*60)
        print("Testing Drop Folder")
        print("="*60)

        drop_folder = self.vault_path.parent.parent / 'Drop_Folder'
        exists = drop_folder.exists() and drop_folder.is_dir()
        self.log_test("Drop_Folder", exists)

    def test_write_permissions(self):
        """Test write permissions"""
        print("\n" + "="*60)
        print("Testing Write Permissions")
        print("="*60)

        test_folders = [
            self.vault_path / 'Needs_Action',
            self.vault_path / 'Logs',
            self.vault_path / 'Done'
        ]

        for folder in test_folders:
            try:
                test_file = folder / f'test_{datetime.now().timestamp()}.txt'
                test_file.write_text('test')
                test_file.unlink()
                self.log_test(f"Write: {folder.name}", True)
            except Exception as e:
                self.log_test(f"Write: {folder.name}", False, str(e))

    def test_create_sample_task(self):
        """Create a sample task to test workflow"""
        print("\n" + "="*60)
        print("Creating Sample Task")
        print("="*60)

        try:
            sample_task = self.vault_path / 'Needs_Action' / 'TEST_sample_task.md'
            content = f"""---
type: test_task
created: {datetime.now().isoformat()}
status: pending
priority: low
---

## Test Task

This is a sample task created by the system test script.

### Objective
Verify that the task processing workflow is functional.

### Actions
- [ ] Read this task
- [ ] Process with /vault-manager
- [ ] Move to Done folder

### Notes
This task can be safely deleted after testing.
"""
            sample_task.write_text(content, encoding='utf-8')
            self.log_test("Create sample task", True,
                         f"Created: {sample_task.name}")
        except Exception as e:
            self.log_test("Create sample task", False, str(e))

    def generate_report(self):
        """Generate test report"""
        print("\n" + "="*60)
        print("Test Summary")
        print("="*60)

        total = len(self.results)
        passed = sum(1 for r in self.results if r['passed'])
        failed = total - passed

        print(f"\nTotal Tests: {total}")
        print(f"Passed: {passed}")
        print(f"Failed: {failed}")
        print(f"Success Rate: {(passed/total*100):.1f}%")

        if failed > 0:
            print("\nFailed Tests:")
            for result in self.results:
                if not result['passed']:
                    print(f"  - {result['test']}")
                    if result['message']:
                        print(f"    {result['message']}")

        # Save report
        report_path = self.vault_path / 'Logs' / f'test_report_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump({
                'timestamp': datetime.now().isoformat(),
                'total': total,
                'passed': passed,
                'failed': failed,
                'success_rate': passed/total*100,
                'results': self.results
            }, f, indent=2)

        print(f"\nDetailed report saved to: {report_path}")

        return failed == 0

    def run_all_tests(self):
        """Run all tests"""
        print("="*60)
        print("AI Employee System Test - Gold Tier")
        print("="*60)
        print(f"Vault Path: {self.vault_path}")
        print(f"Test Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

        # Run all tests
        self.test_vault_structure()
        self.test_core_files()
        self.test_watchers()
        self.test_orchestrator()
        self.test_skills()
        self.test_drop_folder()
        self.test_write_permissions()
        self.test_create_sample_task()

        # Generate report
        all_passed = self.generate_report()

        if all_passed:
            print("\n[SUCCESS] All tests passed! System is ready.")
            print("\nNext steps:")
            print("1. Start orchestrator: python AI_Employee_Vault/orchestrator.py AI_Employee_Vault/vault")
            print("2. Open vault in Obsidian")
            print("3. Process sample task with: claude /vault-manager")
        else:
            print("\n[WARNING] Some tests failed. Please fix issues before proceeding.")

        return all_passed


def main():
    """Main test function"""
    if len(sys.argv) < 2:
        print("Usage: python test_system.py <vault_path>")
        print("Example: python test_system.py AI_Employee_Vault/vault")
        sys.exit(1)

    vault_path = sys.argv[1]
    tester = SystemTester(vault_path)
    success = tester.run_all_tests()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
