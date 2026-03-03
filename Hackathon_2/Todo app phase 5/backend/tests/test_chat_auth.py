"""
Manual Test Script: Authentication Scenarios

This script tests authentication and authorization for the chat endpoint.

Test Scenarios:
1. Request without JWT token (should return 401)
2. Request with invalid JWT token (should return 401)
3. Request with valid token but mismatched user_id (should return 403)
4. Request with valid token and matching user_id (should return 200)

Prerequisites:
- FastAPI server running on http://localhost:8001
- Valid JWT token from authentication
- User exists in database

Usage:
    python backend/test_chat_auth.py
"""

import requests
import json
import sys

# Configuration
BASE_URL = "http://localhost:8001"
USER_ID = "test-user-id"  # Replace with actual user ID from your JWT token
DIFFERENT_USER_ID = "different-user-id"  # Different user ID for mismatch test
JWT_TOKEN = "your-jwt-token-here"  # Replace with actual JWT token

def test_no_token():
    """Test request without JWT token."""
    print("=" * 80)
    print("TEST 1: Request Without JWT Token (Expect 401)")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
    headers = {
        "Content-Type": "application/json"
    }
    payload = {
        "message": "Show me my tasks"
    }

    print(f"\nRequest URL: {url}")
    print(f"Request Headers: {json.dumps(headers, indent=2)}")
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


def test_invalid_token():
    """Test request with invalid JWT token."""
    print("\n\n" + "=" * 80)
    print("TEST 2: Request With Invalid JWT Token (Expect 401)")
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
    """Test request with valid token but mismatched user_id."""
    print("\n\n" + "=" * 80)
    print("TEST 3: Request With User ID Mismatch (Expect 403)")
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


def test_valid_auth():
    """Test request with valid token and matching user_id."""
    print("\n\n" + "=" * 80)
    print("TEST 4: Request With Valid Auth (Expect 200)")
    print("=" * 80)

    url = f"{BASE_URL}/api/{USER_ID}/chat"
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

    print("\nSending request...")
    try:
        response = requests.post(url, headers=headers, json=payload)

        print(f"\nResponse Status: {response.status_code}")

        if response.status_code == 200:
            response_data = response.json()
            print(f"\nResponse Body: {json.dumps(response_data, indent=2)}")
            print("\n" + "=" * 80)
            print("TEST PASSED ✓ - Got expected 200 OK")
            print("=" * 80)
        else:
            print(f"\nResponse Body: {response.text}")
            print("\n" + "=" * 80)
            print(f"TEST FAILED ✗ - Expected 200, got {response.status_code}")
            print("=" * 80)

    except Exception as e:
        print(f"\nError: {e}")
        print("\n" + "=" * 80)
        print("TEST FAILED ✗ - Exception")
        print("=" * 80)


def test_empty_message():
    """Test request with empty message."""
    print("\n\n" + "=" * 80)
    print("TEST 5: Request With Empty Message (Expect 400)")
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

        if response.status_code == 400 or response.status_code == 422:
            print("\n" + "=" * 80)
            print(f"TEST PASSED ✓ - Got expected {response.status_code} (validation error)")
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


if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("CHAT ENDPOINT TEST SUITE - AUTHENTICATION")
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
    test_no_token()
    test_invalid_token()
    test_user_id_mismatch()
    test_valid_auth()
    test_empty_message()

    print("\n" + "=" * 80)
    print("TEST SUITE COMPLETE")
    print("=" * 80)
