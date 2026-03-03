"""
Simple script to test if backend is accessible on different ports
"""
import requests
import json

def test_port(port):
    print(f"\n{'='*80}")
    print(f"Testing port {port}")
    print('='*80)

    # Test health endpoint
    try:
        response = requests.get(f"http://localhost:{port}/health", timeout=2)
        print(f"[HEALTH] Status: {response.status_code}")
        print(f"[HEALTH] Response: {json.dumps(response.json(), indent=2)}")
    except Exception as e:
        print(f"[HEALTH] Error: {e}")
        return False

    # Test OpenAPI spec
    try:
        response = requests.get(f"http://localhost:{port}/openapi.json", timeout=2)
        if response.status_code == 200:
            spec = response.json()
            print(f"\n[OPENAPI] Title: {spec.get('info', {}).get('title')}")
            print(f"[OPENAPI] Version: {spec.get('info', {}).get('version')}")

            paths = list(spec.get('paths', {}).keys())
            print(f"[OPENAPI] Total endpoints: {len(paths)}")

            # Check for key endpoints
            has_auth = any('auth' in p for p in paths)
            has_chat = any('chat' in p for p in paths)
            has_tasks = any('task' in p for p in paths)

            print(f"[OPENAPI] Has auth endpoints: {has_auth}")
            print(f"[OPENAPI] Has chat endpoints: {has_chat}")
            print(f"[OPENAPI] Has task endpoints: {has_tasks}")

            if has_auth and has_chat:
                print(f"\n[SUCCESS] This appears to be the correct FastAPI backend!")
                return True
            else:
                print(f"\n[WARNING] This may not be the correct backend")
                return False
    except Exception as e:
        print(f"\n[OPENAPI] Error: {e}")
        return False

# Test common ports
for port in [8000, 8001, 8002, 3000]:
    test_port(port)

print(f"\n{'='*80}")
print("Port scan complete")
print('='*80)
