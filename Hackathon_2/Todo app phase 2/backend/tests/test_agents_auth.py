"""
Test Gemini authentication using OpenAI Agents SDK
"""
import asyncio
import os
from dotenv import load_dotenv
from agents import AsyncOpenAI

load_dotenv()

async def test_auth():
    """Test if Gemini API authentication works with agents package"""
    gemini_api_key = os.getenv("GEMINI_API_KEY")

    if not gemini_api_key:
        print("ERROR: GEMINI_API_KEY not found in .env")
        return False

    print(f"Testing with API key: {gemini_api_key[:10]}...")

    try:
        # Create client using agents package (same as user's working code)
        client = AsyncOpenAI(
            api_key=gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

        print("Client created successfully")

        # Test a simple API call
        response = await client.chat.completions.create(
            model="gemini-2.5-flash",
            messages=[{"role": "user", "content": "Say hello"}],
            max_tokens=50
        )

        print("API call successful!")
        print(f"Response: {response.choices[0].message.content}")
        return True

    except Exception as e:
        print(f"ERROR: {type(e).__name__}: {e}")
        return False

if __name__ == "__main__":
    success = asyncio.run(test_auth())
    if success:
        print("\n✓ Authentication test PASSED")
    else:
        print("\n✗ Authentication test FAILED")
