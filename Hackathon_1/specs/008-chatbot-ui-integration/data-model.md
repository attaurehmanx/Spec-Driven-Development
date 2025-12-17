# Data Model: RAG Pipeline - Simple Chatbot UI Integration

## Component State Model

### ChatbotUI Component State
- **question** (string): User input text for the question being asked
  - Validation: Non-empty after trimming whitespace
  - Default: Empty string
- **response** (string): Response received from the backend
  - Validation: None (backend provides formatted response)
  - Default: Empty string
- **isLoading** (boolean): Flag indicating if a request is in progress
  - Validation: Boolean value only
  - Default: false
- **error** (string): Error message if request fails
  - Validation: None (backend provides error message)
  - Default: null
- **selectedText** (string): Text selected on the current page (optional)
  - Validation: None (optional context)
  - Default: Empty string

## API Data Models

### Request Model: QueryRequest
- **question** (string, required): The user's question
- **selectedText** (string, optional): Selected text from current page for context
- **timestamp** (string, optional): Client-side timestamp for the request

### Response Model: QueryResponse
- **response** (string, required): The answer from the RAG system
- **sources** (array of objects, optional): Source documents used in response
  - **title** (string): Source document title
  - **url** (string): Source document URL
  - **content** (string): Relevant content snippet
- **timestamp** (string, optional): Server-side timestamp of response

### Error Response Model: ErrorResponse
- **error** (string, required): Error message
- **code** (string, optional): Error code for client handling
- **details** (object, optional): Additional error details for debugging

## UI Interaction Models

### UserInteractionEvent
- **type** (string): "question_submitted", "response_displayed", "error_shown"
- **payload** (object): Event-specific data
- **timestamp** (string): When the event occurred

## Component Input/Output

### Component Props
- **backendUrl** (string, optional): URL of the FastAPI backend (defaults to environment config)
- **showSelectedTextHint** (boolean, optional): Whether to show hint about selected text feature
- **initialHeight** (string, optional): Initial height of the component (e.g., "300px")
- **placeholder** (string, optional): Placeholder text for the input field