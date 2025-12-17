# Quickstart Guide: RAG Pipeline - Simple Chatbot UI Integration

## Development Setup

### Prerequisites
- Node.js 18+
- npm or yarn
- Docusaurus project set up
- Running FastAPI RAG backend

### Installation Steps

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install dependencies**
   ```bash
   npm install
   ```

3. **Set up environment variables**
   Create a `.env` file in the root directory:
   ```env
   REACT_APP_BACKEND_URL=http://localhost:8000
   ```

   For production:
   ```env
   REACT_APP_BACKEND_URL=https://your-backend-domain.com
   ```

4. **Run the development server**
   ```bash
   npm run dev
   # or
   yarn dev
   ```

## Usage

### Embedding the Chatbot Component

1. **Import the component in your Docusaurus MDX file**:
   ```jsx
   import ChatbotUI from '@site/src/components/ChatbotUI/ChatbotUI';

   <ChatbotUI />
   ```

2. **Or use the Docusaurus wrapper**:
   ```jsx
   import ChatbotWrapper from '@site/docs/components/ChatbotWrapper';

   <ChatbotWrapper />
   ```

### Component Properties
- `backendUrl` (optional): Custom backend URL
- `placeholder` (optional): Custom input placeholder text
- `showSelectedTextHint` (optional): Show hint about selected text feature

## API Configuration

The component communicates with the FastAPI backend at the configured URL. The backend must expose:
- POST `/api/query` - for submitting questions
- GET `/api/health` - for health checks

## Testing

1. **Run unit tests**:
   ```bash
   npm test
   ```

2. **Manual testing**:
   - Enter a question in the input field
   - Verify response appears correctly
   - Test with selected text context
   - Verify error handling

## Building for Production

```bash
npm run build
```

The component will be bundled with the Docusaurus site.