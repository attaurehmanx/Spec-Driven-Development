"""
Manual Test Script: Error Handling

This script tests comprehensive error handling for the chat endpoint.

Test Scenarios:
1. Empty message (400 Bad Request)
2. Message exceeding length limit (400 Bad Request)
3. Invalid conversation_id (404 Not Found)
4. Missing JWT token (401 Unauthorized)
5. Invalid JWT token (401 Unauthorized)
6. User ID mismatch (403 Forbidden)
7. Malformed request body (422 Unprocessable Entity)

Prerequisites:
- FastAPI server running on http://localhost:8001
- Valid JWT token from authentication
- User exists in database

Usage:
    python backend/test_chat_errors.py
"""

import requests
import json
import sys

# Configuration
BASE_URL = "http://localhost:8001"
USER_ID = "test-user-id"  # Replace with actual user ID from your JWT token
DIFFERENT_USER_ID = "different-user-id"  # Different user ID for mismatch test
JWT_TOKEN = "your-jwt-token-here"  # Replace with actual JWT token


def test_empty_message():
    """Test error handling for empty message."""
    print("=" * 80)
    print("TEST 1: Empty Message (Expect 400 or 422)")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    payload = {
        "message": ""
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Body: {json.dumps(payload, indent=2)}")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")
        print(f"Response Body: {response.text}")

        if response.status_code in [400, 422]:
            print("\n" + "=" * 80)
            print(f"TEST PASSED ✓ - Got expected {response.status_code}")
            print("=" * 80)
        else:
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Expected 400/422, got {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


def test_whitespace_only_message():
    """Test error handling for whitespace-only message."""
    print("\n\n" + "=" * 80)
    print("TEST 2: Whitespace-Only Message (Expect 400 or 422)")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    payload = {
        "message": "   \n\t   "
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Body: {json.dumps(payload, indent=2)}")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")
        print(f"Response Body: {response.text}")

        if response.status_code in [400, 422]:
            print("\n" + "=" * 80)
            print(f"TEST PASSED ✓ - Got expected {response.status_code}")
            print("=" * 80)
        else:
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Expected 400/422, got {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


def test_message_too_long():
    """Test error handling for message exceeding length limit."""
    print("\n\n" + "=" * 80)
    print("TEST 3: Message Exceeding Length Limit (Expect 400 or 422)")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    # Create a message longer than 10,000 characters
    long_message = "A" * 10001
    payload = {
        "message": long_message
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Body: message with {len(long_message)} characters")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")
        print(f"Response Body: {response.text}")

        if response.status_code in [400, 422]:
            print("\n" + "=" * 80)
            print(f"TEST PASSED ✓ - Got expected {response.status_code}")
            print("=" * 80)
        else:
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Expected 400/422, got {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


def test_invalid_conversation_id():
    """Test error handling for invalid conversation_id."""
    print("\n\n" + "=" * 80)
    print("TEST 4: Invalid Conversation ID (Expect 404)")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    payload = {
        "message": "Show me my tasks",
        "conversation_id": 999999999  # Non-existent conversation ID
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


def test_missing_jwt_token():
    """Test error handling for missing JWT token."""
    print("\n\n" + "=" * 80)
    print("TEST 5: Missing JWT Token (Expect 401)")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Content-Type": "application/json"
    }
    payload = {
        "message": "Show me my tasks"
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Headers: No Authorization header")
    print(f"Request Body: {json.dumps(payload, indent=2)}")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")
        print(f"Response Body: {response.text}")

        if response.status_code == 401:
            print("\n" + "=" * 80)
            print("TEST PASSED ✓ - Got expected 401 Unauthorized")
            print("=" * 80)
        else:
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Expected 401, got {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


def test_invalid_jwt_token():
    """Test error handling for invalid JWT token."""
    print("\n\n" + "=" * 80)
    print("TEST 6: Invalid JWT Token (Expect 401)")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": "Bearer invalid-token-12345",
        "Content-Type": "application/json"
    }
    payload = {
        "message": "Show me my tasks"
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Headers: Authorization: Bearer invalid-token-12345")
    print(f"Request Body: {json.dumps(payload, indent=2)}")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")
        print(f"Response Body: {response.text}")

        if response.status_code == 401:
            print("\n" + "=" * 80)
            print("TEST PASSED ✓ - Got expected 401 Unauthorized")
            print("=" * 80)
        else:
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Expected 401, got {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


def test_user_id_mismatch():
    """Test error handling for user_id mismatch."""
    print("\n\n" + "=" * 80)
    print("TEST 7: User ID Mismatch (Expect 403)")
    print("=" * 80)

    url = f"{BASE_URL}/api/{DIFFERENT_USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    payload = {
        "message": "Show me my tasks"
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Headers: Authorization: Bearer {JWT_TOKEN[:20]}...")
    print(f"Request Body: {json.dumps(payload, indent=2)}")
    print(f"\nNote: Token is for user '{USER_ID}' but URL has '{DIFFERENT_USER_ID}'")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")
        print(f"Response Body: {response.text}")

        if response.status_code == 403:
            print("\n" + "=" * 80)
            print("TEST PASSED ✓ - Got expected 403 Forbidden")
            print("=" * 80)
        else:
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Expected 403, got {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


def test_malformed_request_body():
    """Test error handling for malformed request body."""
    print("\n\n" + "=" * 80)
    print("TEST 8: Malformed Request Body (Expect 422)")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    # Missing required 'message' field
    payload = {
        "conversation_id": 123
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Body: {json.dumps(payload, indent=2)}")
    print(f"\nNote: Missing required 'message' field")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")
        print(f"Response Body: {response.text}")

        if response.status_code == 422:
            print("\n" + "=" * 80)
            print("TEST PASSED ✓ - Got expected 422 Unprocessable Entity")
            print("=" * 80)
        else:
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Expected 422, got {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


def test_negative_conversation_id():
    """Test error handling for negative conversation_id."""
    print("\n\n" + "=" * 80)
    print("TEST 9: Negative Conversation ID (Expect 422)")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Authorization": f"Bearer {JWT_TOKEN}",
        "Content-Type": "application/json"
    }
    payload = {
        "message": "Show me my tasks",
        "conversation_id": -1
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Body: {json.dumps(payload, indent=2)}")

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")
        print(f"Response Body: {response.text}")

        if response.status_code == 422:
            print("\n" + "=" * 80)
            print("TEST PASSED ✓ - Got expected 422 Unprocessable Entity")
            print("=" * 80)
        else:
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Expected 422, got {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("CHAT ENDPOINT TEST SUITE - ERROR HANDLING")
    print("=" * 80)

    # Check if JWT token is set
    if JWT_TOKEN == "your-jwt-token-here":
        print("\n⚠️  WARNING: Please set JWT_TOKEN and USER_ID variables")
        print("You can get these by:")
        print("1. Sign up/sign in via the auth endpoint")
        print("2. Copy the token and user ID from the response")
        print("3. Update JWT_TOKEN and USER_ID variables in this script")
        sys.exit(1)

    # Run all tests
    test_empty_message()
    test_whitespace_only_message()
    test_message_too_long()
    test_invalid_conversation_id()
    test_missing_jwt_token()
    test_invalid_jwt_token()
    test_user_id_mismatch()
    test_malformed_request_body()
    test_negative_conversation_id()

    print("\n" + "=" * 80)
    print("TEST SUITE COMPLETE")
    print("=" * 80)
    print("\nSummary:")
    print("- Test 1: Empty message validation")
    print("- Test 2: Whitespace-only message validation")
    print("- Test 3: Message length limit validation")
    print("- Test 4: Invalid conversation_id handling")
    print("- Test 5: Missing JWT token handling")
    print("- Test 6: Invalid JWT token handling")
    print("- Test 7: User ID mismatch handling")
    print("- Test 8: Malformed request body handling")
    print("- Test 9: Negative conversation_id validation")
