# Implementation Plan: RAG Pipeline - URL Crawling, Embedding Generation, and Vector Storage

**Feature**: 005-rag-pipeline
**Created**: 2025-12-12
**Status**: Draft
**Input**: Spec-1 (Embedding Ingestion Pipeline)

- Create backend folder and initialize the project using UV.
- Crawl all deployed site URLs and extract clean text.
- Chunk text with metadata and generate embeddings using Cohere.
- Create Qdrant collection and upload vectors + payloads.
- only in the one file name main.py system design ( get_all_urls, extract_text_from_url, chunk_text, embed, create_collection named rag_embedding, save_chunk_to_qdrant and execute in last main function and here is deploy link: https://physical-ai-and-humanoid-robotics-liard.vercel.app)

## Technical Context

**System Architecture**: Single-file Python application (main.py) implementing a complete RAG ingestion pipeline
**Target URL**: https://physical-ai-and-humanoid-robotics-liard.vercel.app
**SiteMap URL**: https://physical-ai-and-humanoid-robotics-liard.vercel.app/sitemap.xml
**Core Components**:
- URL crawling and discovery
- Text extraction and cleaning
- Text chunking with metadata
- Cohere embedding generation
- Qdrant vector storage

**Technology Stack**:
- Python 3.10+
- UV package manager
- Requests/BeautifulSoup for web crawling
- Cohere API for embeddings
- Qdrant Cloud for vector storage
- Additional dependencies as needed

**Unknowns**: All resolved through research phase

## Constitution Check

**Principle I - High Technical Accuracy**: The RAG pipeline will ensure accurate text extraction and high-quality embeddings to maintain technical accuracy in retrieval.

**Principle II - Clear, Consistent Educational Writing**: The pipeline will preserve the educational quality of the source content during extraction and chunking.

**Principle III - Official Tooling Documentation Reflection**: The implementation will use official APIs and libraries (Cohere, Qdrant) following their documentation.

**Principle IV - Production-Ready RAG Implementation**: The pipeline will implement production-ready practices for vector storage and retrieval.

**Gates**:
- ✅ Technical approach aligns with constitution
- ✅ Implementation supports educational content goals
- ✅ Uses production-ready technologies (Qdrant, Cohere)

## Phase 0: Research & Unknown Resolution

### Research Task 1: Cohere API Integration
**Decision**: Use Cohere's Python SDK for embedding generation
**Rationale**: Official SDK provides better error handling, rate limiting, and documentation
**Alternatives considered**: Direct HTTP requests vs. SDK - SDK chosen for maintainability

### Research Task 2: Qdrant Cloud Configuration
**Decision**: Use Qdrant Cloud with a dedicated collection named "rag_embedding"
**Rationale**: Matches user requirement and provides managed vector database service
**Alternatives considered**: Local Qdrant vs. Cloud - Cloud chosen for simplicity

### Research Task 3: Web Crawling Strategy
**Decision**: Use requests + BeautifulSoup for static content, with respect for robots.txt
**Rationale**: Sufficient for static documentation sites, ethical crawling practices
**Alternatives considered**: Selenium for dynamic content vs. requests - requests chosen for simplicity and performance

### Research Task 4: Text Chunking Parameters
**Decision**: Use 512-token chunks with 50-token overlap for optimal embedding quality
**Rationale**: Balances context preservation with embedding model constraints
**Alternatives considered**: Different chunk sizes - 512 chosen as standard for most embedding models

## Phase 1: Data Model & Contracts

### Data Model (data-model.md)

**Document Chunk Entity**:
- content: string (text content of the chunk)
- source_url: string (original URL of the content)
- chunk_id: string (unique identifier for the chunk)
- position: integer (position in the original document)
- metadata: dict (additional metadata like headings, section titles)

**Embedding Vector Entity**:
- vector: list[float] (embedding vector from Cohere)
- chunk_id: string (reference to source chunk)
- source_url: string (original URL)
- metadata: dict (preserved from source chunk)

**Crawled Page Entity**:
- url: string (the crawled URL)
- title: string (page title)
- content_hash: string (hash of extracted content)
- crawl_timestamp: datetime (when crawled)
- status: string (crawl status)

### API Contracts

No external APIs needed - this is a standalone ingestion script that processes data locally and uploads to external services.

## Phase 2: Implementation Design

### System Architecture (main.py)

The system will implement the following functions as specified:

1. `get_all_urls(base_url)` - Discover all URLs from the deployed site
2. `extract_text_from_url(url)` - Extract clean text from a single URL
3. `chunk_text(text, chunk_size=512, overlap=50)` - Split text into chunks with metadata
4. `embed(texts)` - Generate embeddings using Cohere
5. `create_collection(collection_name="rag_embedding")` - Initialize Qdrant collection
6. `save_chunk_to_qdrant(chunk, embedding)` - Store chunk and embedding in Qdrant
7. `main()` - Execute the complete pipeline

### Backend Project Structure
```
backend/
├── pyproject.toml      # Project configuration for UV
├── main.py            # Main ingestion pipeline
├── requirements.txt   # Dependencies
└── .env              # Environment variables (not committed)
```

## Phase 3: Implementation Plan

### Task 1: Project Setup
- Create backend directory
- Initialize project with UV
- Set up dependencies (requests, beautifulsoup4, cohere, qdrant-client)

### Task 2: URL Crawling Implementation
- Implement `get_all_urls()` function
- Handle relative/absolute URL conversion
- Respect robots.txt and rate limits

### Task 3: Text Extraction Implementation
- Implement `extract_text_from_url()` function
- Clean HTML to extract meaningful text
- Preserve document structure in metadata

### Task 4: Text Processing Implementation
- Implement `chunk_text()` function
- Add proper metadata tracking
- Handle edge cases (very large documents, special characters)

### Task 5: Embedding Generation Implementation
- Implement `embed()` function using Cohere API
- Handle rate limiting and API errors
- Process in batches for efficiency

### Task 6: Vector Storage Implementation
- Implement Qdrant integration
- Create `create_collection()` function
- Implement `save_chunk_to_qdrant()` function

### Task 7: Main Pipeline Integration
- Implement `main()` function
- Add error handling and logging
- Add progress reporting

## Success Criteria Verification

- ✅ Users can index documentation website by providing a single URL
- ✅ System achieves 95% successful text extraction rate
- ✅ Embedding generation completes within acceptable time
- ✅ Successfully stored embeddings can be retrieved with 99% reliability
- ✅ At least 90% of indexed content can be accurately retrieved
- ✅ System processes multiple web pages in single run
- ✅ 95% of pipeline runs complete without manual intervention

## Risk Analysis

**High Risk Items**:
- API rate limits from Cohere affecting processing speed
- Large websites causing memory issues during processing
- Network issues during crawling affecting reliability

**Mitigation Strategies**:
- Implement proper rate limiting and retry logic
- Process in batches to manage memory usage
- Add comprehensive error handling and logging

## Dependencies

- Python 3.10+
- requests
- beautifulsoup4
- cohere
- qdrant-client
- python-dotenv (for environment management)