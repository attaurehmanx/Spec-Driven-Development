# Data Model: RAG Pipeline

## Document Chunk Entity
- **content**: string (text content of the chunk)
- **source_url**: string (original URL of the content)
- **chunk_id**: string (unique identifier for the chunk, typically UUID)
- **position**: integer (position in the original document, for ordering)
- **metadata**: dict (additional metadata like headings, section titles, page title)

## Embedding Vector Entity
- **vector**: list[float] (embedding vector from Cohere, dimension depends on model)
- **chunk_id**: string (reference to source chunk for retrieval)
- **source_url**: string (original URL for attribution)
- **metadata**: dict (preserved from source chunk, including document structure info)

## Crawled Page Entity
- **url**: string (the crawled URL)
- **title**: string (page title extracted from HTML)
- **content_hash**: string (SHA256 hash of extracted content for deduplication)
- **crawl_timestamp**: datetime (ISO format, when crawled)
- **status**: string (crawl status: "success", "error", "skipped")
- **word_count**: integer (number of words in extracted content)

## Qdrant Point Structure
- **id**: string (unique point ID in Qdrant)
- **vector**: list[float] (the embedding vector)
- **payload**: dict (contains source_url, content, metadata, etc.)