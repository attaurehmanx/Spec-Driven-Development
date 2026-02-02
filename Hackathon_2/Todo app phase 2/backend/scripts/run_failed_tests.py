"""
Run Failed Tests with Extended Delays

This script runs only the tests that failed due to rate limiting,
with 60-second delays between each test to avoid API limits.
"""

import asyncio
import sys
import os
from datetime import datetime
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add backend to path
sys.path.insert(0, os.path.dirname(__file__))

# Import the test runner class
from run_all_tests import TestRunner


async def main():
    """Run only the failed tests with extended delays."""
    runner = TestRunner()

    print("\n")
    print("+" + "="*68 + "+")
    print("|" + " "*8 + "RUNNING FAILED TESTS WITH EXTENDED DELAYS" + " "*19 + "|")
    print("+" + "="*68 + "+")
    print("\nNOTE: Running 5 failed tests with 60-second delays")
    print("NOTE: Total estimated time: ~5-6 minutes\n")

    # Run failed tests with 60-second delays
    print("Running T027 - 'the first one' reference...")
    await runner.test_t027_first_one_reference()
    print("Waiting 60 seconds before next test...\n")
    await asyncio.sleep(60)

    print("Running T035 - Task not found error...")
    await runner.test_t035_task_not_found_error()
    print("Waiting 60 seconds before next test...\n")
    await asyncio.sleep(60)

    print("Running T036 - Ambiguous request clarification...")
    await runner.test_t036_ambiguous_request_clarification()
    print("Waiting 60 seconds before next test...\n")
    await asyncio.sleep(60)

    print("Running T041 - Today interpretation...")
    await runner.test_t041_today_interpretation()
    print("Waiting 60 seconds before next test...\n")
    await asyncio.sleep(60)

    print("Running T042 - Tomorrow interpretation...")
    await runner.test_t042_tomorrow_interpretation()

    # Generate report
    print("\n\n")
    print("+" + "="*68 + "+")
    print("|" + " "*15 + "FAILED TESTS RETRY REPORT" + " "*28 + "|")
    print("+" + "="*68 + "+")
    print(f"\nTest Execution Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"User ID: {runner.user_id}")
    print(f"Total Tests: {len(runner.results)}")

    passed = sum(1 for r in runner.results if r.passed)
    failed = sum(1 for r in runner.results if not r.passed and not r.error)
    errors = sum(1 for r in runner.results if r.error)

    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    print(f"Errors: {errors}")
    print(f"Success Rate: {(passed/len(runner.results)*100):.1f}%")

    print("\n" + "="*70)
    print("DETAILED RESULTS")
    print("="*70)

    for result in runner.results:
        status = "PASS" if result.passed else ("ERROR" if result.error else "FAIL")
        print(f"\n{result.test_id} - {result.test_name}")
        print(f"  User Story: {result.user_story}")
        print(f"  Status: {status}")
        print(f"  Message: {result.message}")
        print(f"  Iterations: {result.response.iterations if result.response else 0}")
        print(f"  Execution Time: {result.execution_time_ms:.2f}ms")
        if result.response:
            print(f"  Agent Response: {result.response.final_response[:200]}...")

    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)

    if passed == len(runner.results):
        print("PASS: All failed tests now passing!")
    else:
        print(f"FAIL: {failed + errors} test(s) still failing")

    print("="*70)

    return 0 if passed == len(runner.results) else 1


if __name__ == '__main__':
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
