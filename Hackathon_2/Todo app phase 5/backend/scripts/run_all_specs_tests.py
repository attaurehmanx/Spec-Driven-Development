#!/usr/bin/env python3
"""
Master Test Runner for Specs 005, 006, and 007
Starts backend on port 8001 and runs all automated tests
"""

import subprocess
import sys
import time
import requests
import os
from pathlib import Path

# ANSI color codes
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
BOLD = '\033[1m'
RESET = '\033[0m'

def print_header(text):
    """Print a formatted header"""
    print(f"\n{BLUE}{BOLD}{'='*70}{RESET}")
    print(f"{BLUE}{BOLD}{text.center(70)}{RESET}")
    print(f"{BLUE}{BOLD}{'='*70}{RESET}\n")

def print_success(text):
    """Print success message"""
    print(f"{GREEN}✓ {text}{RESET}")

def print_error(text):
    """Print error message"""
    print(f"{RED}✗ {text}{RESET}")

def print_info(text):
    """Print info message"""
    print(f"{YELLOW}ℹ {text}{RESET}")

def check_backend_health(port=8001, max_retries=30, retry_delay=2):
    """Check if backend is healthy"""
    url = f"http://localhost:{port}/health"

    for attempt in range(max_retries):
        try:
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                data = response.json()
                print_success(f"Backend is healthy on port {port}")
                print_info(f"Service: {data.get('service', 'unknown')}")
                return True
        except requests.exceptions.RequestException:
            if attempt < max_retries - 1:
                print(f"Waiting for backend... (attempt {attempt + 1}/{max_retries})")
                time.sleep(retry_delay)
            else:
                print_error(f"Backend not responding after {max_retries} attempts")
                return False

    return False

def start_backend(port=8001):
    """Start the FastAPI backend"""
    print_header("Starting Backend Server")

    backend_dir = Path(__file__).parent

    print_info(f"Starting backend on port {port}...")
    print_info("This will open a new terminal window")
    print_info("Keep the terminal window open while tests run")

    # Start backend in a new terminal window
    if sys.platform == "win32":
        # Windows
        cmd = f'start cmd /k "cd /d {backend_dir} && uvicorn main:app --reload --port {port}"'
        subprocess.Popen(cmd, shell=True)
    else:
        # Linux/Mac
        cmd = f'gnome-terminal -- bash -c "cd {backend_dir} && uvicorn main:app --reload --port {port}; exec bash"'
        subprocess.Popen(cmd, shell=True)

    print_info("Waiting for backend to start...")
    time.sleep(5)  # Give it time to start

    # Check if backend is healthy
    if check_backend_health(port):
        print_success("Backend started successfully!")
        return True
    else:
        print_error("Failed to start backend")
        return False

def run_spec_005_tests():
    """Run Spec 005 (AI Agent Service) tests"""
    print_header("Running Spec 005: AI Agent Service Tests")

    test_script = Path(__file__).parent / "run_all_tests.py"

    if not test_script.exists():
        print_error(f"Test script not found: {test_script}")
        return False

    print_info("Running 8 test scenarios for AI Agent Service...")
    print_info("This will test: add_task, list_tasks, pronoun resolution, etc.")
    print()

    try:
        result = subprocess.run(
            [sys.executable, str(test_script)],
            cwd=test_script.parent,
            capture_output=False,
            text=True
        )

        if result.returncode == 0:
            print_success("Spec 005 tests completed successfully!")
            return True
        else:
            print_error(f"Spec 005 tests failed with return code {result.returncode}")
            return False
    except Exception as e:
        print_error(f"Error running Spec 005 tests: {e}")
        return False

def run_spec_006_tests():
    """Run Spec 006 (Chat Endpoint) tests"""
    print_header("Running Spec 006: Chat Endpoint Tests")

    test_script = Path(__file__).parent / "run_all_chat_tests.py"

    if not test_script.exists():
        print_error(f"Test script not found: {test_script}")
        return False

    print_info("Running 9 test scenarios for Chat Endpoint...")
    print_info("This will test: authentication, persistence, agent integration, etc.")
    print()

    try:
        result = subprocess.run(
            [sys.executable, str(test_script)],
            cwd=test_script.parent,
            capture_output=False,
            text=True
        )

        if result.returncode == 0:
            print_success("Spec 006 tests completed successfully!")
            return True
        else:
            print_error(f"Spec 006 tests failed with return code {result.returncode}")
            return False
    except Exception as e:
        print_error(f"Error running Spec 006 tests: {e}")
        return False

def print_spec_007_instructions():
    """Print instructions for manual Spec 007 testing"""
    print_header("Spec 007: Frontend Chat Interface - Manual Testing Required")

    print(f"{YELLOW}Spec 007 requires manual browser testing.{RESET}")
    print()
    print("To test Spec 007:")
    print()
    print("1. Start the frontend (in a new terminal):")
    print(f"   {BLUE}cd \"Z:\\phse 33\\frontend-app\"{RESET}")
    print(f"   {BLUE}npm run dev{RESET}")
    print()
    print("2. Open your browser to: http://localhost:3000")
    print()
    print("3. Follow the testing guide:")
    print(f"   {BLUE}Z:\\phse 33\\specs\\007-frontend-chat-interface\\CHEAT_SHEET.md{RESET}")
    print()
    print("4. Execute 12 test scenarios:")
    print("   - Basic chat interaction")
    print("   - Multi-turn conversations")
    print("   - Task creation/update/deletion")
    print("   - Auto-refresh verification")
    print("   - Responsive design")
    print("   - Accessibility")
    print("   - Performance metrics")
    print()
    print(f"5. Record results in: {BLUE}TEST_RESULTS_TEMPLATE.md{RESET}")
    print()
    print(f"6. Update tasks.md (lines 281-292) to mark tests as {GREEN}[x]{RESET}")
    print()

def print_final_summary(spec_005_passed, spec_006_passed):
    """Print final test summary"""
    print_header("Test Execution Summary")

    print(f"\n{BOLD}Automated Tests:{RESET}")
    print(f"  Spec 005 (AI Agent Service): {GREEN + '✓ PASSED' if spec_005_passed else RED + '✗ FAILED'}{RESET}")
    print(f"  Spec 006 (Chat Endpoint):    {GREEN + '✓ PASSED' if spec_006_passed else RED + '✗ FAILED'}{RESET}")

    print(f"\n{BOLD}Manual Tests:{RESET}")
    print(f"  Spec 007 (Frontend Chat):    {YELLOW}⏳ PENDING (requires manual testing){RESET}")

    print(f"\n{BOLD}Next Steps:{RESET}")
    if spec_005_passed and spec_006_passed:
        print(f"  {GREEN}1. All automated tests passed!{RESET}")
        print(f"  2. Update tasks.md files to mark completed tests as [x]")
        print(f"  3. Proceed with manual testing for Spec 007 (see instructions above)")
        print(f"  4. After all tests complete, run: {BLUE}/sp.git.commit_pr{RESET}")
    else:
        print(f"  {RED}1. Review failed tests and fix issues{RESET}")
        print(f"  2. Re-run this script after fixes")

    print()

def main():
    """Main test execution"""
    print_header("Master Test Runner - Specs 005, 006, 007")

    print(f"{BOLD}This script will:{RESET}")
    print("  1. Start the backend on port 8001")
    print("  2. Run automated tests for Spec 005 (AI Agent Service)")
    print("  3. Run automated tests for Spec 006 (Chat Endpoint)")
    print("  4. Provide instructions for manual testing of Spec 007 (Frontend)")
    print()

    input(f"{YELLOW}Press Enter to continue...{RESET}")

    # Step 1: Start backend
    if not start_backend(port=8001):
        print_error("Cannot proceed without backend running")
        print_info("Please start the backend manually and try again:")
        print_info("  cd \"Z:\\phse 33\\backend\"")
        print_info("  uvicorn main:app --reload --port 8001")
        sys.exit(1)

    # Step 2: Run Spec 005 tests
    spec_005_passed = run_spec_005_tests()

    # Step 3: Run Spec 006 tests
    spec_006_passed = run_spec_006_tests()

    # Step 4: Print Spec 007 instructions
    print_spec_007_instructions()

    # Final summary
    print_final_summary(spec_005_passed, spec_006_passed)

    # Exit code
    if spec_005_passed and spec_006_passed:
        print_success("Automated testing complete!")
        sys.exit(0)
    else:
        print_error("Some automated tests failed")
        sys.exit(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Test execution interrupted by user{RESET}")
        sys.exit(1)
    except Exception as e:
        print_error(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
