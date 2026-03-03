"""
Check OpenAPI spec to see what endpoints are actually available
"""
import requests
import json

BASE_URL = "http://localhost:8001"

print("Fetching OpenAPI specification...")
print("=" * 80)

try:
    response = requests.get(f"{BASE_URL}/openapi.json")
    if response.status_code == 200:
        spec = response.json()

        print(f"\nAPI Title: {spec.get('info', {}).get('title')}")
        print(f"API Version: {spec.get('info', {}).get('version')}")
        print(f"\nAvailable endpoints:")
        print("-" * 80)

        paths = spec.get('paths', {})
        for path, methods in sorted(paths.items()):
            for method in methods.keys():
                if method.upper() in ['GET', 'POST', 'PUT', 'DELETE', 'PATCH']:
                    print(f"  {method.upper():6} {path}")

        print("\n" + "=" * 80)

        # Check for auth-related endpoints
        auth_endpoints = [p for p in paths.keys() if 'auth' in p.lower()]
        if auth_endpoints:
            print("\nAuth-related endpoints found:")
            for endpoint in auth_endpoints:
                print(f"  {endpoint}")
        else:
            print("\nNo auth-related endpoints found in OpenAPI spec")

        # Check for chat endpoints
        chat_endpoints = [p for p in paths.keys() if 'chat' in p.lower()]
        if chat_endpoints:
            print("\nChat-related endpoints found:")
            for endpoint in chat_endpoints:
                print(f"  {endpoint}")
        else:
            print("\nNo chat-related endpoints found in OpenAPI spec")

    else:
        print(f"Failed to fetch OpenAPI spec: {response.status_code}")

except Exception as e:
    print(f"Error: {e}")
