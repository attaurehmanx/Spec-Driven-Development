# Quickstart: RAG Pipeline Validation - Retrieval & Validation

## Prerequisites

- Python 3.10+
- Qdrant vector database running with existing collection
- Cohere API key
- Required Python packages (see requirements.txt)

## Setup

1. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

2. **Configure environment**:
   Create/update `.env` file with:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_HOST=localhost
   QDRANT_PORT=6333
   COLLECTION_NAME=your_collection_name
   ```

3. **Verify Qdrant connection**:
   Ensure your Qdrant instance is running and the collection exists with proper schema.

## Usage

1. **Run retrieval validation**:
   ```bash
   cd backend
   python retrieve.py --query "your test query here"
   ```

2. **Run validation tests**:
   ```bash
   python retrieve.py --validate
   ```

3. **Test with filters**:
   ```bash
   python retrieve.py --query "filtered query" --filters '{"source": "manual"}'
   ```

## Example

```python
from retrieve import RAGValidator

# Initialize validator
validator = RAGValidator()

# Perform a query
results = validator.retrieve("What is the ROS 2 architecture?")

# Validate results
validation_result = validator.validate_retrieval(results, expected_content=["ROS 2", "DDS", "nodes"])

print(f"Precision: {validation_result.precision}")
print(f"Success: {validation_result.success}")
```

## Validation Tests

The system includes several built-in validation tests:

- **Accuracy Test**: Compares retrieval results against known good answers
- **Latency Test**: Ensures query response times meet performance requirements
- **Metadata Test**: Validates that document metadata was properly preserved
- **Coverage Test**: Ensures the system can retrieve content across different document types

## Troubleshooting

- **Connection errors**: Verify Qdrant is running and accessible
- **Empty results**: Check that the collection contains indexed documents
- **Low quality results**: Verify embeddings were generated correctly during ingestion
- **API errors**: Confirm Cohere API key is valid and has sufficient quota