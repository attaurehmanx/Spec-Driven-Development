# Data Model: RAG Pipeline Validation - Retrieval & Validation

## Core Entities

### QueryRequest
- **query_text** (string): The natural language query from the user
- **filters** (dict, optional): Metadata filters to apply during search
- **top_k** (int, default=5): Number of results to return
- **query_embedding** (list[float], optional): Vector representation of the query

### RetrievedChunk
- **content** (string): The text content of the retrieved document chunk
- **id** (string): Unique identifier for the document chunk
- **score** (float): Similarity score between query and chunk
- **metadata** (dict): Associated metadata (source, page number, etc.)
- **vector** (list[float], optional): Vector embedding of the chunk

### RetrievalResult
- **query** (QueryRequest): The original query that was processed
- **results** (list[RetrievedChunk]): Ranked list of retrieved chunks
- **search_time_ms** (float): Time taken to perform the search
- **total_candidates** (int): Total number of candidates considered
- **validated** (bool): Whether the retrieval meets quality standards

### ValidationResult
- **test_name** (string): Name/description of the validation test
- **query** (string): The test query used
- **expected_results** (list[string]): Expected documents or content
- **actual_results** (list[RetrievedChunk]): Actual retrieval results
- **precision** (float): Precision score of the retrieval
- **recall** (float): Recall score of the retrieval
- **success** (bool): Whether validation passed

## Data Flow

1. **Query Input**: User provides a natural language query
2. **Embedding**: Query is converted to vector using Cohere
3. **Search**: Vector search performed in Qdrant with optional filters
4. **Ranking**: Results ranked by similarity score
5. **Output**: Retrieved chunks with scores and metadata returned
6. **Validation**: Results compared against expected outcomes for quality assurance

## Schema Validation Rules

- Query text must not exceed 1000 characters
- top_k must be between 1 and 100
- Scores must be between 0 and 1
- Required metadata fields: source, created_date
- Vector dimensions must match collection schema
- Filters must conform to Qdrant payload schema