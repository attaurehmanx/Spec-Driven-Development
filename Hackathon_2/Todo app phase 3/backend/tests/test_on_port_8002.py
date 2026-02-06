"""
Test if backend is running on port 8002 and run tests
"""
import requests
import time
import subprocess
import sys

def check_backend(port):
    """Check if backend is running"""
    try:
        response = requests.get(f"http://localhost:{port}/health", timeout=2)
        if response.status_code == 200:
            data = response.json()
            print(f"[SUCCESS] Backend is running on port {port}")
            print(f"[INFO] Response: {data}")
            return True
    except:
        pass
    return False

def main():
    print("=" * 80)
    print("CHECKING BACKEND AND RUNNING TESTS")
    print("=" * 80)

    # Check port 8002
    print("\n[STEP 1] Checking port 8002...")
    if check_backend(8002):
        print("[SUCCESS] Backend found on port 8002")

        # Update test script to use port 8002
        print("\n[STEP 2] Running comprehensive test suite on port 8002...")

        # Run the test suite with port 8002
        import run_all_chat_tests
        tester = run_all_chat_tests.ChatEndpointTester(base_url="http://localhost:8002")
        success = tester.run_all_tests()

        return success
    else:
        print("[ERROR] Backend not found on port 8002")
        print("\nPlease start the backend manually:")
        print('  cd "Z:\\phse 33\\backend"')
        print("  uvicorn main:app --reload --port 8002")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
