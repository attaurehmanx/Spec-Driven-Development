"""
Quick test of the agent service with new API key
"""
import asyncio
from dotenv import load_dotenv

load_dotenv()

from services.agent_service import run_agent

async def test_agent_service():
    """Test the agent service with a simple request"""
    print("Testing agent service with new API key...")

    try:
        # Test with the test user
        response = await run_agent(
            user_id="test-user-123",
            message_history=[
                {"role": "user", "content": "Add a task to buy milk"}
            ]
        )

        print(f"\nStatus: {response.status}")
        print(f"Iterations: {response.iterations}")
        print(f"Response: {response.final_response[:200]}...")
        print(f"Execution time: {response.execution_time_ms:.2f}ms")

        if response.status == "completed":
            print("\nSUCCESS: Agent service is working correctly!")
            return True
        else:
            print(f"\nFAILED: Status was {response.status}")
            if response.error:
                print(f"Error: {response.error}")
            return False

    except Exception as e:
        print(f"\nERROR: {type(e).__name__}: {e}")
        return False

if __name__ == "__main__":
    success = asyncio.run(test_agent_service())
    exit(0 if success else 1)
