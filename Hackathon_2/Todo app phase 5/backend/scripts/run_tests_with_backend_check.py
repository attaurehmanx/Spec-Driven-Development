"""
Wrapper script to check backend and run comprehensive tests
"""
import requests
import time
import sys
import subprocess
import os

def check_backend(port):
    """Check if backend is running on specified port"""
    try:
        response = requests.get(f"http://localhost:{port}/health", timeout=2)
        if response.status_code == 200:
            data = response.json()
            print(f"[INFO] Backend is running on port {port}")
            print(f"[INFO] Service: {data}")
            return True
    except:
        pass
    return False

def main():
    print("=" * 80)
    print("CHAT ENDPOINT TESTING - BACKEND CHECK AND TEST EXECUTION")
    print("=" * 80)

    # Check port 8001 first (our backend)
    print("\n[STEP 1] Checking for FastAPI backend on port 8001...")
    if check_backend(8001):
        print("[SUCCESS] FastAPI backend found on port 8001")
        port = 8001
    else:
        print("[INFO] Backend not found on port 8001")
        print("[STEP 2] Checking for backend on port 8000...")

        # Check if it's the correct backend on 8000
        try:
            response = requests.get("http://localhost:8001/openapi.json", timeout=2)
            if response.status_code == 200:
                spec = response.json()
                title = spec.get('info', {}).get('title', '')

                if 'Task Management' in title or 'chat' in str(spec.get('paths', {})):
                    print("[SUCCESS] FastAPI backend found on port 8000")
                    port = 8000
                else:
                    print(f"[ERROR] Wrong service on port 8000: {title}")
                    print("[ERROR] Cannot proceed - FastAPI backend not running")
                    print("\nTo start the backend, run:")
                    print('  cd "Z:\\phse 33\\backend"')
                    print("  uvicorn main:app --reload --port 8001")
                    return False
        except:
            print("[ERROR] No backend found on port 8000")
            print("[ERROR] Cannot proceed - FastAPI backend not running")
            print("\nTo start the backend, run:")
            print('  cd "Z:\\phse 33\\backend"')
            print("  uvicorn main:app --reload --port 8001")
            return False

    # Update the test script to use the correct port
    print(f"\n[STEP 3] Running comprehensive test suite on port {port}...")
    print("=" * 80)

    # Import and run the test suite with the correct port
    sys.path.insert(0, os.path.dirname(__file__))

    # Modify the BASE_URL in the test module
    import run_all_chat_tests
    run_all_chat_tests.ChatEndpointTester.base_url = f"http://localhost:{port}"

    # Create tester instance with correct port
    tester = run_all_chat_tests.ChatEndpointTester(base_url=f"http://localhost:{port}")
    success = tester.run_all_tests()

    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
