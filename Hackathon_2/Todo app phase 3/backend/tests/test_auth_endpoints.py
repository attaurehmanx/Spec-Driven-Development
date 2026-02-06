"""
Quick diagnostic script to test auth endpoints
"""
import requests
import json

BASE_URL = "http://localhost:8001"

print("Testing authentication endpoints...")
print("=" * 80)

# Test 1: Check root endpoint
print("\n1. Testing root endpoint (GET /):")
try:
    response = requests.get(f"{BASE_URL}/")
    print(f"   Status: {response.status_code}")
    print(f"   Response: {json.dumps(response.json(), indent=2)}")
except Exception as e:
    print(f"   Error: {e}")

# Test 2: Check health endpoint
print("\n2. Testing health endpoint (GET /health):")
try:
    response = requests.get(f"{BASE_URL}/health")
    print(f"   Status: {response.status_code}")
    print(f"   Response: {json.dumps(response.json(), indent=2)}")
except Exception as e:
    print(f"   Error: {e}")

# Test 3: Try to register a user
print("\n3. Testing register endpoint (POST /auth/register):")
try:
    response = requests.post(
        f"{BASE_URL}/auth/register",
        json={
            "email": "diagnostic_test@example.com",
            "password": "TestPassword123!",
            "name": "Diagnostic Test"
        }
    )
    print(f"   Status: {response.status_code}")
    print(f"   Response: {response.text}")
except Exception as e:
    print(f"   Error: {e}")

# Test 4: Try to login
print("\n4. Testing login endpoint (POST /auth/login):")
try:
    response = requests.post(
        f"{BASE_URL}/auth/login",
        json={
            "email": "diagnostic_test@example.com",
            "password": "TestPassword123!"
        }
    )
    print(f"   Status: {response.status_code}")
    print(f"   Response: {response.text}")

    if response.status_code == 200:
        data = response.json()
        token = data.get("access_token")
        print(f"\n   Got token: {token[:30]}...")

        # Test 5: Verify token
        print("\n5. Testing verify-token endpoint (GET /auth/verify-token):")
        verify_response = requests.get(
            f"{BASE_URL}/auth/verify-token",
            headers={"Authorization": f"Bearer {token}"}
        )
        print(f"   Status: {verify_response.status_code}")
        print(f"   Response: {json.dumps(verify_response.json(), indent=2)}")

except Exception as e:
    print(f"   Error: {e}")

print("\n" + "=" * 80)
print("Diagnostic complete")
