"""
Comprehensive Test Runner for AI Agent Service

This script runs all manual tests for the AI Agent Service and generates
a detailed test report with pass/fail status for each test case.

Usage:
    python run_all_tests.py

Prerequisites:
    - Backend database is accessible
    - GEMINI_API_KEY is set in .env
    - Test user exists in database (or will be created)
"""

import asyncio
import sys
import os
from datetime import datetime
from typing import List, Dict, Any
from dotenv import load_dotenv

# Load environment variables from .env file BEFORE importing agent_service
load_dotenv()

# Add backend to path
sys.path.insert(0, os.path.dirname(__file__))

from services.agent_service import run_agent


class TestResult:
    """Container for test execution results."""
    def __init__(self, test_id: str, test_name: str, user_story: str):
        self.test_id = test_id
        self.test_name = test_name
        self.user_story = user_story
        self.passed = False
        self.message = ""
        self.response = None
        self.error = None
        self.execution_time_ms = 0.0


class TestRunner:
    """Executes all agent service tests and generates report."""

    def __init__(self):
        self.results: List[TestResult] = []
        self.user_id = "test-user-123"

    async def run_test(self, test_id: str, test_name: str, user_story: str,
                       message_history: List[Dict[str, str]],
                       validation_func) -> TestResult:
        """Execute a single test case."""
        result = TestResult(test_id, test_name, user_story)

        print(f"\n{'='*70}")
        print(f"Running: {test_id} - {test_name}")
        print(f"{'='*70}")

        try:
            response = await run_agent(message_history, self.user_id)
            result.response = response
            result.execution_time_ms = response.execution_time_ms

            # Run validation
            result.passed, result.message = validation_func(response)

            print(f"Status: {response.status}")
            print(f"Iterations: {response.iterations}")
            print(f"Time: {response.execution_time_ms:.2f}ms")
            print(f"Response: {response.final_response[:200]}...")
            print(f"\nValidation: {'PASS' if result.passed else 'FAIL'} - {result.message}")

        except Exception as e:
            result.error = str(e)
            result.message = f"Exception: {type(e).__name__}: {e}"
            print(f"ERROR: {result.message}")

        self.results.append(result)
        return result

    # ========================================================================
    # User Story 1 Tests: Basic Agent Conversation
    # ========================================================================

    async def test_t020_add_task_tool_call(self):
        """T020: Verify agent correctly calls add_task tool with proper parameters."""

        message_history = [
            {"role": "user", "content": "Create a task to buy groceries"}
        ]

        def validate(response):
            if response.status != "completed":
                return False, f"Status was {response.status}, expected completed"

            response_lower = response.final_response.lower()
            if "task" not in response_lower:
                return False, "Response doesn't mention 'task'"

            if not ("created" in response_lower or "added" in response_lower):
                return False, "Response doesn't confirm task creation"

            if "groceries" not in response_lower:
                return False, "Response doesn't mention 'groceries'"

            return True, "Agent correctly called add_task and confirmed creation"

        return await self.run_test(
            "T020",
            "Verify agent calls add_task tool with proper parameters",
            "US1",
            message_history,
            validate
        )

    async def test_t021_list_tasks_tool_call(self):
        """T021: Verify agent correctly calls list_tasks tool and formats response."""

        message_history = [
            {"role": "user", "content": "Show me my tasks"}
        ]

        def validate(response):
            if response.status != "completed":
                return False, f"Status was {response.status}, expected completed"

            response_lower = response.final_response.lower()
            if "task" not in response_lower:
                return False, "Response doesn't mention 'task'"

            # Should either list tasks or say no tasks
            has_list = any(word in response_lower for word in ["have", "list", "tasks", "pending", "completed"])
            if not has_list:
                return False, "Response doesn't appear to list tasks"

            return True, "Agent correctly called list_tasks and formatted response"

        return await self.run_test(
            "T021",
            "Verify agent calls list_tasks tool and formats response",
            "US1",
            message_history,
            validate
        )

    # ========================================================================
    # User Story 2 Tests: Multi-Turn Conversation with Context
    # ========================================================================

    async def test_t026_pronoun_resolution(self):
        """T026: Verify agent correctly references previous messages when user says 'Mark it as done'."""

        message_history = [
            {"role": "user", "content": "Create a task to call mom"},
            {"role": "assistant", "content": "I've created a task titled 'Call mom' for you."},
            {"role": "user", "content": "Mark it as complete"}
        ]

        def validate(response):
            if response.status != "completed":
                return False, f"Status was {response.status}, expected completed"

            response_lower = response.final_response.lower()

            # Should mention completion or marking as done
            if not any(word in response_lower for word in ["complete", "done", "marked", "finished"]):
                return False, "Response doesn't confirm task completion"

            # Should reference the task (mom or call)
            if not any(word in response_lower for word in ["mom", "call", "task"]):
                return False, "Response doesn't reference the correct task"

            return True, "Agent correctly resolved 'it' to 'Call mom' task"

        return await self.run_test(
            "T026",
            "Verify agent references previous messages for pronoun resolution",
            "US2",
            message_history,
            validate
        )

    async def test_t027_first_one_reference(self):
        """T027: Verify agent correctly identifies 'the first one' from previous list_tasks response."""

        message_history = [
            {"role": "user", "content": "Show me my tasks"},
            {"role": "assistant", "content": "You have 3 tasks:\n1. Buy groceries (pending)\n2. Call dentist (pending)\n3. Finish report (completed)"},
            {"role": "user", "content": "Delete the first one"}
        ]

        def validate(response):
            if response.status != "completed":
                return False, f"Status was {response.status}, expected completed"

            response_lower = response.final_response.lower()

            # Should mention deletion
            if not any(word in response_lower for word in ["delete", "removed", "deleted"]):
                return False, "Response doesn't confirm deletion"

            # Should reference groceries (the first task)
            if "groceries" not in response_lower and "first" not in response_lower:
                return False, "Response doesn't reference the first task (groceries)"

            return True, "Agent correctly identified 'the first one' as 'Buy groceries'"

        return await self.run_test(
            "T027",
            "Verify agent identifies 'the first one' from previous list",
            "US2",
            message_history,
            validate
        )

    # ========================================================================
    # User Story 3 Tests: Error Handling and User Guidance
    # ========================================================================

    async def test_t035_task_not_found_error(self):
        """T035: Verify agent explains 'Task not found' error in natural language."""

        message_history = [
            {"role": "user", "content": "Mark task 99999 as complete"}
        ]

        def validate(response):
            if response.status != "completed":
                return False, f"Status was {response.status}, expected completed"

            response_lower = response.final_response.lower()

            # Should explain task not found in user-friendly way
            error_phrases = ["not found", "couldn't find", "doesn't exist", "can't find", "unable to find"]
            if not any(phrase in response_lower for phrase in error_phrases):
                return False, "Response doesn't explain task not found error"

            # Should be polite and helpful
            if "error" in response_lower or "exception" in response_lower:
                return False, "Response contains technical error terms"

            return True, "Agent explained task not found error in user-friendly language"

        return await self.run_test(
            "T035",
            "Verify agent explains 'Task not found' error naturally",
            "US3",
            message_history,
            validate
        )

    async def test_t036_ambiguous_request_clarification(self):
        """T036: Verify agent asks clarifying questions when request is ambiguous."""

        message_history = [
            {"role": "user", "content": "Update my task"}
        ]

        def validate(response):
            if response.status != "completed":
                return False, f"Status was {response.status}, expected completed"

            response_lower = response.final_response.lower()

            # Should ask clarifying question
            clarifying_indicators = ["which", "what", "?", "specify", "tell me", "need to know"]
            if not any(indicator in response_lower for indicator in clarifying_indicators):
                return False, "Response doesn't ask clarifying question"

            # Should mention task or what needs clarification
            if "task" not in response_lower:
                return False, "Response doesn't mention task"

            return True, "Agent asked clarifying question for ambiguous request"

        return await self.run_test(
            "T036",
            "Verify agent asks clarifying questions for ambiguous requests",
            "US3",
            message_history,
            validate
        )

    # ========================================================================
    # User Story 4 Tests: Time-Aware Task Management
    # ========================================================================

    async def test_t041_today_interpretation(self):
        """T041: Verify agent correctly interprets 'today' and filters tasks by current date."""

        message_history = [
            {"role": "user", "content": "Show me tasks due today"}
        ]

        def validate(response):
            if response.status != "completed":
                return False, f"Status was {response.status}, expected completed"

            response_lower = response.final_response.lower()

            # Should reference today or current date
            if not any(word in response_lower for word in ["today", "task", datetime.utcnow().strftime("%Y-%m-%d")]):
                return False, "Response doesn't reference today or tasks"

            return True, "Agent correctly interpreted 'today' for task filtering"

        return await self.run_test(
            "T041",
            "Verify agent interprets 'today' correctly",
            "US4",
            message_history,
            validate
        )

    async def test_t042_tomorrow_interpretation(self):
        """T042: Verify agent correctly interprets 'tomorrow' and creates task with tomorrow's date."""

        message_history = [
            {"role": "user", "content": "Create a task to call the dentist tomorrow"}
        ]

        def validate(response):
            if response.status != "completed":
                return False, f"Status was {response.status}, expected completed"

            response_lower = response.final_response.lower()

            # Should mention task creation
            if not any(word in response_lower for word in ["created", "added", "task"]):
                return False, "Response doesn't confirm task creation"

            # Should reference dentist
            if "dentist" not in response_lower:
                return False, "Response doesn't mention dentist"

            # Should reference tomorrow or future date
            if "tomorrow" not in response_lower:
                return False, "Response doesn't mention tomorrow"

            return True, "Agent correctly interpreted 'tomorrow' for task creation"

        return await self.run_test(
            "T042",
            "Verify agent interprets 'tomorrow' correctly",
            "US4",
            message_history,
            validate
        )

    # ========================================================================
    # Report Generation
    # ========================================================================

    def generate_report(self):
        """Generate comprehensive test report."""
        print("\n\n")
        print("+" + "="*68 + "+")
        print("|" + " "*15 + "AI AGENT SERVICE TEST REPORT" + " "*25 + "|")
        print("+" + "="*68 + "+")
        print(f"\nTest Execution Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"User ID: {self.user_id}")
        print(f"Total Tests: {len(self.results)}")

        # Count results by status
        passed = sum(1 for r in self.results if r.passed)
        failed = sum(1 for r in self.results if not r.passed and not r.error)
        errors = sum(1 for r in self.results if r.error)

        print(f"Passed: {passed}")
        print(f"Failed: {failed}")
        print(f"Errors: {errors}")
        print(f"Success Rate: {(passed/len(self.results)*100):.1f}%")

        # Group by user story
        print("\n" + "="*70)
        print("RESULTS BY USER STORY")
        print("="*70)

        for us in ["US1", "US2", "US3", "US4"]:
            us_results = [r for r in self.results if r.user_story == us]
            if us_results:
                us_passed = sum(1 for r in us_results if r.passed)
                print(f"\n{us}: {us_passed}/{len(us_results)} passed")
                for result in us_results:
                    status = "PASS" if result.passed else ("ERROR" if result.error else "FAIL")
                    print(f"  {status} {result.test_id}: {result.test_name}")
                    if not result.passed:
                        print(f"       -> {result.message}")

        # Detailed results
        print("\n" + "="*70)
        print("DETAILED TEST RESULTS")
        print("="*70)

        for result in self.results:
            print(f"\n{result.test_id} - {result.test_name}")
            print(f"  User Story: {result.user_story}")
            print(f"  Status: {'PASS' if result.passed else ('ERROR' if result.error else 'FAIL')}")
            print(f"  Message: {result.message}")
            if result.response:
                print(f"  Iterations: {result.response.iterations}")
                print(f"  Execution Time: {result.execution_time_ms:.2f}ms")
                print(f"  Agent Response: {result.response.final_response[:150]}...")

        # Summary
        print("\n" + "="*70)
        print("SUMMARY")
        print("="*70)

        if passed == len(self.results):
            print("ALL TESTS PASSED - Agent service is working correctly!")
        elif errors > 0:
            print(f"ERROR: {errors} test(s) encountered errors - Check configuration and dependencies")
        else:
            print(f"FAIL: {failed} test(s) failed - Review agent behavior and system prompt")

        print("\n" + "="*70)

        return passed == len(self.results)

    async def run_all_tests(self):
        """Execute all test cases."""
        print("\n")
        print("+" + "="*68 + "+")
        print("|" + " "*10 + "STARTING AI AGENT SERVICE TEST SUITE" + " "*22 + "|")
        print("+" + "="*68 + "+")
        print("\nNOTE: Ensure GEMINI_API_KEY is set in backend/.env")
        print("NOTE: Ensure database is accessible and test user exists")
        print("NOTE: Adding 12-second delays between tests to avoid rate limiting\n")

        # Run all tests with delays to avoid rate limiting
        await self.test_t020_add_task_tool_call()
        print("Waiting 12 seconds before next test...")
        await asyncio.sleep(12)

        await self.test_t021_list_tasks_tool_call()
        print("Waiting 12 seconds before next test...")
        await asyncio.sleep(12)

        await self.test_t026_pronoun_resolution()
        print("Waiting 12 seconds before next test...")
        await asyncio.sleep(12)

        await self.test_t027_first_one_reference()
        print("Waiting 12 seconds before next test...")
        await asyncio.sleep(12)

        await self.test_t035_task_not_found_error()
        print("Waiting 12 seconds before next test...")
        await asyncio.sleep(12)

        await self.test_t036_ambiguous_request_clarification()
        print("Waiting 12 seconds before next test...")
        await asyncio.sleep(12)

        await self.test_t041_today_interpretation()
        print("Waiting 12 seconds before next test...")
        await asyncio.sleep(12)

        await self.test_t042_tomorrow_interpretation()

        # Generate report
        all_passed = self.generate_report()

        return 0 if all_passed else 1


async def main():
    """Main entry point."""
    runner = TestRunner()
    exit_code = await runner.run_all_tests()
    sys.exit(exit_code)


if __name__ == "__main__":
    asyncio.run(main())
