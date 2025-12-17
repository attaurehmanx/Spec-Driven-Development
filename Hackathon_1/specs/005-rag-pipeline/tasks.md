# Implementation Tasks: RAG Pipeline

**Feature**: 005-rag-pipeline
**Created**: 2025-12-12
**Status**: Draft

## Implementation Strategy

MVP approach: Implement the core RAG ingestion pipeline in a single main.py file with all required functions. Focus on crawling the target URL, extracting text, generating embeddings, and storing in Qdrant.

## Dependencies

- Python 3.10+
- UV package manager
- Cohere API access
- Qdrant Cloud access

## Phase 1: Setup Tasks

- [x] T001 Create backend directory structure
- [x] T002 Initialize Python project with UV and create pyproject.toml
- [x] T003 Add dependencies (requests, beautifulsoup4, cohere, qdrant-client, python-dotenv) to pyproject.toml
- [x] T004 Create main.py file with imports and configuration setup

## Phase 2: Foundational Tasks

- [x] T005 Set up environment variables loading (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
- [x] T006 Create Qdrant client connection function
- [x] T007 Implement error handling and logging utilities
- [x] T008 [P] Create helper functions for text cleaning and validation

## Phase 3: [US1] URL Crawling Implementation

- [x] T009 [US1] Implement get_all_urls() function to discover site URLs
- [x] T010 [US1] Add sitemap support to crawl https://physical-ai-and-humanoid-robotics-liard.vercel.app/sitemap.xml
- [x] T011 [US1] Implement URL validation and filtering logic
- [x] T012 [US1] Add rate limiting to respect robots.txt and avoid overwhelming the server

## Phase 4: [US2] Text Extraction Implementation

- [x] T013 [US2] Implement extract_text_from_url() function
- [x] T014 [US2] Add HTML parsing to extract clean text content
- [x] T015 [US2] Implement content filtering to remove navigation, ads, and non-content elements
- [x] T016 [US2] Add metadata extraction (title, headings) to preserve document structure

## Phase 5: [US3] Text Chunking and Embedding

- [x] T017 [US3] Implement chunk_text() function with 512-token chunks and 50-token overlap
- [x] T018 [US3] Add metadata preservation in chunks (source URL, position, document structure)
- [x] T019 [US3] Implement embed() function using Cohere API
- [x] T020 [US3] Add batch processing for efficient embedding generation

## Phase 6: [US4] Vector Storage Implementation

- [x] T021 [US4] Implement create_collection() function to create "rag_embedding" collection
- [x] T022 [US4] Implement save_chunk_to_qdrant() function to store embeddings with metadata
- [x] T023 [US4] Add vector ID generation and payload structure per data model
- [x] T024 [US4] Implement error handling for Qdrant operations

## Phase 7: [US5] Main Pipeline Integration

- [x] T025 [US5] Implement main() function to orchestrate the complete pipeline
- [x] T026 [US5] Add progress reporting and status updates
- [x] T027 [US5] Implement pipeline error handling and recovery
- [x] T028 [US5] Add command-line interface for URL input

## Phase 8: Polish & Cross-Cutting Concerns

- [x] T029 Add comprehensive error logging and debugging information
- [x] T030 Create .env.example file with template for environment variables
- [x] T031 Add README.md with setup and usage instructions
- [x] T032 Implement basic tests for each major function
- [x] T033 Optimize performance for large document sets

## Task Dependencies

- US1 (URL Crawling) → US2 (Text Extraction) → US3 (Embedding) → US4 (Storage) → US5 (Integration)
- Foundational tasks (T005-T008) must complete before any user story tasks

## Parallel Execution Opportunities

- [P] tasks can be executed in parallel when they work on different components
- Text extraction and embedding can be parallelized per document
- Multiple chunks can be embedded in batches

## Independent Test Criteria

- **US1**: Can discover and return all valid URLs from the target website
- **US2**: Can extract clean text from any provided URL, filtering out non-content elements
- **US3**: Can chunk text appropriately and generate valid embeddings using Cohere
- **US4**: Can create Qdrant collection and store embeddings with metadata successfully
- **US5**: Complete pipeline runs end-to-end with proper error handling and reporting