"""
Manual Test Script: Multi-turn Conversation

This script tests multi-turn conversation functionality with context preservation.

Test Scenarios:
1. Create a new conversation with an initial message
2. Send follow-up messages that reference previous context
3. Verify the AI agent maintains context and resolves pronouns correctly
4. Test conversation history is properly loaded

Prerequisites:
- FastAPI server running on http://localhost:8001
- Valid JWT token from authentication
- User exists in database

Usage:
    python backend/test_chat_multiturn.py
"""

import requests
import json
import sys
import time

# Configuration
BASE_URL = "http://localhost:8001"
USER_ID = "test-user-id"  # Replace with actual user ID from your JWT token
JWT_TOKEN = "your-jwt-token-here"  # Replace with actual JWT token


def test_create_initial_conversation():
    """Test creating a new conversation with initial message."""
    print("=" * 80)
    print("TEST 1: Create Initial Conversation")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    payload = {
        "message": "Create a task to call mom tomorrow"
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Body: {json.dumps(payload, indent=2)}")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")

        if response.status_code == 200:
            response_data = response.json()
            print(f"\nResponse Body: {json.dumps(response_data, indent=2)}")

            # Validate response
            assert "conversation_id" in response_data, "Missing conversation_id"
            assert "response" in response_data, "Missing response"
            assert "tool_calls" in response_data, "Missing tool_calls"

            conversation_id = response_data["conversation_id"]
            print(f"\n✓ Created conversation {conversation_id}")
            print(f"✓ AI Response: {response_data['response'][:100]}...")
            print(f"✓ Tool calls: {response_data['tool_calls']}")

            print("\n" + "=" * 80)
            print("TEST PASSED ✓")
            print("=" * 80)

            return conversation_id

        else:
            print(f"\nResponse Body: {response.text}")
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Status {response.status_code}")
            print("=" * 80)
            return None

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)
        return None


def test_continue_with_pronoun(conversation_id: int):
    """Test continuing conversation with pronoun reference."""
    print("\n\n" + "=" * 80)
    print("TEST 2: Continue Conversation with Pronoun Reference")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    payload = {
        "message": "Mark it as complete",
        "conversation_id": conversation_id
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Body: {json.dumps(payload, indent=2)}")
    print(f"\nNote: 'it' should refer to the task created in the previous message")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")

        if response.status_code == 200:
            response_data = response.json()
            print(f"\nResponse Body: {json.dumps(response_data, indent=2)}")

            # Validate response
            assert response_data["conversation_id"] == conversation_id, "conversation_id mismatch"
            print(f"\n✓ Same conversation_id: {conversation_id}")
            print(f"✓ AI Response: {response_data['response']}")
            print(f"✓ Tool calls: {response_data['tool_calls']}")

            # Check if agent successfully resolved the pronoun
            if "complete_task" in response_data.get("tool_calls", []):
                print("\n✓ Agent successfully resolved 'it' and called complete_task")
            else:
                print("\n⚠ Agent may not have resolved the pronoun correctly")

            print("\n" + "=" * 80)
            print("TEST PASSED ✓")
            print("=" * 80)

        else:
            print(f"\nResponse Body: {response.text}")
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Status {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


def test_continue_with_reference(conversation_id: int):
    """Test continuing conversation with contextual reference."""
    print("\n\n" + "=" * 80)
    print("TEST 3: Continue Conversation with Contextual Reference")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    payload = {
        "message": "What was that task about?",
        "conversation_id": conversation_id
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Body: {json.dumps(payload, indent=2)}")
    print(f"\nNote: 'that task' should refer to the task from earlier in the conversation")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")

        if response.status_code == 200:
            response_data = response.json()
            print(f"\nResponse Body: {json.dumps(response_data, indent=2)}")

            # Validate response
            assert response_data["conversation_id"] == conversation_id, "conversation_id mismatch"
            print(f"\n✓ Same conversation_id: {conversation_id}")
            print(f"✓ AI Response: {response_data['response']}")

            # Check if response mentions "call mom" or similar context
            response_text = response_data['response'].lower()
            if "mom" in response_text or "call" in response_text:
                print("\n✓ Agent successfully maintained context and referenced the original task")
            else:
                print("\n⚠ Agent response may not have maintained full context")

            print("\n" + "=" * 80)
            print("TEST PASSED ✓")
            print("=" * 80)

        else:
            print(f"\nResponse Body: {response.text}")
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Status {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


def test_invalid_conversation_id():
    """Test error handling for invalid conversation_id."""
    print("\n\n" + "=" * 80)
    print("TEST 4: Invalid Conversation ID")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    payload = {
        "message": "Show me my tasks",
        "conversation_id": 999999  # Non-existent conversation ID
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Body: {json.dumps(payload, indent=2)}")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")
        print(f"Response Body: {response.text}")

        if response.status_code == 404:
            print("\n" + "=" * 80)
            print("TEST PASSED ✓ - Got expected 404 Not Found")
            print("=" * 80)
        else:
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Expected 404, got {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


def test_long_conversation():
    """Test conversation with multiple turns."""
    print("\n\n" + "=" * 80)
    print("TEST 5: Long Multi-turn Conversation")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }

    messages = [
        "Create a task to buy groceries",
        "Add another task to do laundry",
        "Show me all my tasks",
        "Mark the first one as complete",
        "How many tasks do I have left?"
    ]

    conversation_id = None

    for i, message in enumerate(messages, 1):
        print(f"\n--- Turn {i}/{len(messages)} ---")
        print(f"Message: {message}")

        payload = {"message": message}
        if conversation_id:
            payload["conversation_id"] = conversation_id

        try:
            response = requests.post(url, headers=headers, json=payload)

            if response.status_code == 200:
                response_data = response.json()
                conversation_id = response_data["conversation_id"]
                print(f"✓ Response: {response_data['response'][:80]}...")
                print(f"✓ Tool calls: {response_data['tool_calls']}")
            else:
                print(f"✗ Failed with status {response.status_code}")
                print(f"Response: {response.text}")
                break

            # Small delay between requests
            time.sleep(0.5)

        except Exception as e:
            print(f"✗ Error: {e}")
            break

    print("\n" + "=" * 80)
    print("TEST COMPLETED - Check responses above for context preservation")
    print("=" * 80)


if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("CHAT ENDPOINT TEST SUITE - MULTI-TURN CONVERSATIONS")
    print("=" * 80)

    # Check if JWT token is set
    if JWT_TOKEN == "your-jwt-token-here":
        print("\n⚠️  WARNING: Please set JWT_TOKEN and USER_ID variables")
        print("You can get these by:")
        print("1. Sign up/sign in via the auth endpoint")
        print("2. Copy the token and user ID from the response")
        print("3. Update JWT_TOKEN and USER_ID variables in this script")
        sys.exit(1)

    # Test 1: Create initial conversation
    conversation_id = test_create_initial_conversation()

    if conversation_id:
        # Test 2: Continue with pronoun reference
        test_continue_with_pronoun(conversation_id)

        # Test 3: Continue with contextual reference
        test_continue_with_reference(conversation_id)

    # Test 4: Invalid conversation ID
    test_invalid_conversation_id()

    # Test 5: Long multi-turn conversation
    test_long_conversation()

    print("\n" + "=" * 80)
    print("TEST SUITE COMPLETE")
    print("=" * 80)
