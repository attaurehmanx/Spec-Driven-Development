# Quickstart Guide: RAG Pipeline

## Prerequisites
- Python 3.10+
- UV package manager
- Cohere API key
- Qdrant Cloud cluster URL and API key

## Setup
1. Clone the repository
2. Navigate to the backend directory
3. Install dependencies with UV: `uv pip install -r requirements.txt`
4. Create a `.env` file with your API keys:

```
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Run the Pipeline
1. Execute the main script: `python main.py`
2. The pipeline will:
   - Crawl the target website (https://physical-ai-and-humanoid-robotics-liard.vercel.app)
   - Extract text from all discovered URLs
   - Chunk the text with metadata
   - Generate embeddings using Cohere
   - Store everything in Qdrant collection named "rag_embedding"

## Expected Output
- Processed document chunks stored in Qdrant
- Console output showing progress
- Error logs if any issues occur