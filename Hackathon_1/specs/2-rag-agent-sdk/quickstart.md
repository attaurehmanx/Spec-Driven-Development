# Quickstart Guide: RAG Pipeline

## Overview
This guide will help you quickly set up and run the RAG (Retrieval-Augmented Generation) pipeline that uses OpenAI Agents SDK with Qdrant vector database to answer questions about book content.

## Prerequisites
- Python 3.10 or higher
- pip package manager
- Access to OpenAI-compatible API (either OpenAI or configured Gemini endpoint)
- Access to Qdrant vector database
- (Optional) Google AI API key if using Gemini models directly

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

Or install the core dependencies directly:
```bash
pip install openai qdrant-client python-dotenv pydantic fastapi uvicorn
```

### 4. Configure Environment Variables
Create a `.env` file in the project root with the following variables:

```env
# API Keys
OPENAI_API_KEY=your_openai_api_key
# OR if using Gemini via OpenAI-compatible interface:
GEMINI_API_KEY=your_gemini_api_key

# Qdrant Configuration
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_content

# Application Configuration
MODEL_NAME=gemini-pro  # or other compatible model
TRACING_ENABLED=true
LOG_LEVEL=INFO
```

## Basic Usage

### 1. Initialize the Agent
```python
from rag_pipeline import RAGAgent

# Initialize the agent with tracing enabled
agent = RAGAgent(
    model="gemini-pro",
    tracing_enabled=True,
    qdrant_config={
        "url": "your_qdrant_url",
        "collection": "book_content",
        "api_key": "your_qdrant_api_key"
    }
)

# The agent is now ready to process queries
```

### 2. Process a Query
```python
# Ask a question about book content
response = agent.query("What are the key concepts in chapter 3?")

print(f"Response: {response['response']}")
print(f"Retrieved chunks: {len(response['retrieved_chunks'])}")
print(f"Trace ID: {response['trace_id']}")
```

### 3. Using the API
Start the FastAPI server:
```bash
uvicorn main:app --reload --port 8000
```

Then make requests to the API:
```bash
# Initialize an agent
curl -X POST "http://localhost:8000/api/agent/init" \
  -H "Content-Type: application/json" \
  -d '{
    "model": "gemini-pro",
    "tracing_enabled": true,
    "qdrant_config": {
      "url": "your_qdrant_url",
      "collection": "book_content"
    }
  }'

# Process a query
curl -X POST "http://localhost:8000/api/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key concepts in chapter 3?",
    "top_k": 5
  }'
```

## Configuration Options

### Agent Initialization Parameters
- `model`: The model to use (e.g., "gemini-pro", "gpt-4")
- `tracing_enabled`: Whether to enable request tracing (boolean)
- `system_prompt`: Optional custom system prompt
- `qdrant_config`: Configuration for Qdrant connection

### Query Parameters
- `query`: The user's question about book content
- `top_k`: Number of content chunks to retrieve (default: 5)
- `session_id`: Session identifier for follow-up questions
- `agent_id`: Use specific agent instance (optional)

## Example Use Cases

### 1. Simple Question Answering
```python
response = agent.query("What is the main theme of the book?")
print(response['response'])
```

### 2. Chapter-Specific Questions
```python
response = agent.query("Summarize chapter 5")
print(response['response'])
```

### 3. Follow-up Questions in Session
```python
# First question establishes session
response1 = agent.query("What is machine learning?", session_id="session-123")

# Follow-up question uses same context
response2 = agent.query("Can you give me examples?", session_id="session-123")
```

## Troubleshooting

### Common Issues

1. **API Key Errors**: Ensure your API keys are correctly set in the `.env` file
2. **Qdrant Connection**: Verify that your Qdrant URL and credentials are correct
3. **Model Not Found**: Check that the model name is supported by your API provider

### Debugging
Enable detailed logging by setting `LOG_LEVEL=DEBUG` in your `.env` file.

## Next Steps
- Explore the full API documentation at `/docs` when running the server
- Check out the example notebooks in the `examples/` directory
- Review the data models in `data-model.md` for detailed schema information