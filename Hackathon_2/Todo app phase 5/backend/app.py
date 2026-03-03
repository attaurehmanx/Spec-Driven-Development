from main import app

# For Hugging Face Spaces deployment
# The app is imported as 'app' so that the uvicorn server can find it
# when running: uvicorn app:app --host 0.0.0.0 --port 7860