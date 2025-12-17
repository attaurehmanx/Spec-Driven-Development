# Research Summary: RAG Pipeline - Simple Chatbot UI Integration

## Decision: Docusaurus-Compatible React Component Architecture
**Rationale**: Based on the feature requirements, the UI must be embeddable in Docusaurus pages using React components without external UI frameworks. This approach ensures compatibility with the existing Docusaurus documentation site while maintaining the minimal, lightweight requirement.

## Technology Stack Decisions

### React Component Structure
- **Decision**: Create a self-contained ChatbotUI component with minimal dependencies
- **Rationale**: Keeps the component lightweight and compatible with Docusaurus v3
- **Alternatives considered**:
  - Using external UI frameworks (rejected - violates constraint of "no external UI frameworks")
  - Full standalone page (rejected - needs to be embeddable in existing pages)

### Backend Communication
- **Decision**: Use fetch API or Axios for FastAPI backend communication
- **Rationale**: Standard web API approaches that work well in React components
- **Alternatives considered**:
  - Custom HTTP clients (unnecessary complexity for simple requests)
  - WebSocket connections (overkill for simple query-response pattern)

### Text Selection Integration
- **Decision**: Implement DOM text selection API for capturing selected text
- **Rationale**: Native browser API that works well with React components
- **Alternatives considered**:
  - Third-party selection libraries (unnecessary complexity)
  - Manual selection tracking (reinventing browser functionality)

### State Management
- **Decision**: Use React useState and useEffect hooks for component state
- **Rationale**: Built-in React patterns that are sufficient for this simple UI
- **Alternatives considered**:
  - Redux/Zustand (overkill for simple state: question, response, loading)
  - Context API (unnecessary for single component)

### Styling Approach
- **Decision**: CSS Modules for component styling
- **Rationale**: Provides scoped styling without conflicts, compatible with Docusaurus
- **Alternatives considered**:
  - Styled-components (adds dependency, may conflict with Docusaurus)
  - Global CSS (risk of style conflicts)
  - Inline styles (harder to maintain)

## Backend Integration Details

### API Endpoint Structure
- **Decision**: Standard REST API call to FastAPI backend
- **Payload structure**: { question: string, selectedText?: string }
- **Response structure**: { response: string } or { error: string }
- **Error handling**: Network errors, backend errors, timeout handling

### Development Environment Compatibility
- **Decision**: Use environment variables for backend URL configuration
- **Rationale**: Allows local development without hardcoding URLs
- **Local development approach**: Proxy or CORS configuration as needed

## Docusaurus Integration Approach

### Component Embedding
- **Decision**: Create a Docusaurus-compatible wrapper component
- **Rationale**: Ensures proper integration with Docusaurus lifecycle and styling
- **Implementation**: Separate wrapper that can be imported in Docusaurus MDX files

### Navigation Integration
- **Decision**: Provide both inline component and sidebar integration options
- **Rationale**: Supports both "always visible" and "on-demand" access patterns
- **Implementation**: Separate components for different integration points