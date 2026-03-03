"""
Manual Test Script: New Conversation

This script tests creating a new conversation via the chat endpoint.

Test Scenarios:
1. Send a message without conversation_id (should create new conversation)
2. Verify conversation_id is returned
3. Verify AI response is returned
4. Verify tool_calls array is returned

Prerequisites:
- FastAPI server running on http://localhost:8001
- Valid JWT token from authentication
- User exists in database

Usage:
    python backend/test_chat_new_conversation.py
"""

import requests
import json
import sys
import os

# Configuration
BASE_URL = "http://localhost:8001"
USER_ID = "test-user-id"  # Replace with actual user ID from your JWT token
JWT_TOKEN = "your-jwt-token-here"  # Replace with actual JWT token

def test_new_conversation():
    """Test creating a new conversation."""
    print("=" * 80)
    print("TEST: Create New Conversation")
    print("=" * 80)

    # Prepare request
    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    payload = {
        "message": "Show me my tasks"
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Headers: {json.dumps({k: v[:20] + '...' if k == 'Authorization' else v for k, v in headers.items()}, indent=2)}")
    print(f"Request Body: {json.dumps(payload, indent=2)}")

    # Send request
    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")
        print(f"Response Headers: {json.dumps(dict(response.headers), indent=2)}")

        if response.status_code == 200:
            response_data = response.json()
            print(f"\nResponse Body: {json.dumps(response_data, indent=2)}")

            # Validate response structure
            print("\n" + "=" * 80)
            print("VALIDATION")
            print("=" * 80)

            assert "conversation_id" in response_data, "Missing conversation_id"
            assert isinstance(response_data["conversation_id"], int), "conversation_id must be integer"
            assert response_data["conversation_id"] > 0, "conversation_id must be positive"
            print(f"✓ conversation_id: {response_data['conversation_id']}")

            assert "response" in response_data, "Missing response"
            assert isinstance(response_data["response"], str), "response must be string"
            assert len(response_data["response"]) > 0, "response cannot be empty"
            print(f"✓ response: {response_data['response'][:100]}...")

            assert "tool_calls" in response_data, "Missing tool_calls"
            assert isinstance(response_data["tool_calls"], list), "tool_calls must be list"
            print(f"✓ tool_calls: {response_data['tool_calls']}")

            print("\n" + "=" * 80)
            print("TEST PASSED ✓")
            print("=" * 80)

            return response_data["conversation_id"]

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


def test_continue_conversation(conversation_id: int):
    """Test continuing an existing conversation."""
    print("\n\n" + "=" * 80)
    print("TEST: Continue Existing Conversation")
    print("=" * 80)

    # Prepare request
    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    payload = {
        "message": "Create a task to buy milk",
        "conversation_id": conversation_id
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Body: {json.dumps(payload, indent=2)}")

    # Send request
    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")

        if response.status_code == 200:
            response_data = response.json()
            print(f"\nResponse Body: {json.dumps(response_data, indent=2)}")

            # Validate response
            assert response_data["conversation_id"] == conversation_id, "conversation_id should match"
            print(f"✓ Same conversation_id: {conversation_id}")

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


if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("CHAT ENDPOINT TEST SUITE - NEW CONVERSATION")
    print("=" * 80)

    # Check if JWT token is set
    if JWT_TOKEN == "your-jwt-token-here":
        print("\n⚠️  WARNING: Please set JWT_TOKEN variable with a valid token")
        print("You can get a token by:")
        print("1. Sign up/sign in via the auth endpoint")
        print("2. Copy the token from the response")
        print("3. Update JWT_TOKEN variable in this script")
        sys.exit(1)

    # Test 1: Create new conversation
    conversation_id = test_new_conversation()

    # Test 2: Continue conversation (if first test passed)
    if conversation_id:
        test_continue_conversation(conversation_id)

    print("\n" + "=" * 80)
    print("TEST SUITE COMPLETE")
    print("=" * 80)
