# RAG Chatbot Component

A React-based chatbot component that integrates with a RAG (Retrieval Augmented Generation) backend to answer questions about book content.

## Features

- **Question Answering**: Ask questions about book content and receive AI-generated responses
- **Text Selection**: Select text in the book and ask questions constrained to that specific content
- **Conversation History**: Maintains context across multiple interactions
- **Session Management**: Persists conversation state across page refreshes
- **Error Handling**: Graceful handling of backend errors and network issues
- **Accessibility**: Full keyboard navigation and screen reader support
- **Responsive Design**: Works on all device sizes

## Usage

### Basic Integration

```jsx
import ChatbotContainer from './components/Chatbot';

function App() {
  return (
    <div className="app">
      <main>
        {/* Your book content here */}
      </main>
      <ChatbotContainer />
    </div>
  );
}
```

## Functionality

### Asking Questions
1. Type your question in the input field at the bottom
2. Press Enter or click "Send" to submit
3. The response will appear in the chat history

### Using Selected Text
1. Highlight text in the book content
2. The selected text indicator will appear above the chat
3. Check "Answer only from selected text" to constrain responses
4. Ask your question as normal

### Managing Conversations
- Your session persists across page refreshes
- Multiple exchanges maintain context automatically
- Click "Clear Selection" to remove selected text

## API Configuration

The component uses the following environment variable for backend configuration:

- `REACT_APP_API_BASE_URL`: Base URL for the FastAPI backend (defaults to `http://localhost:8000`)

## Accessibility

This component follows WCAG 2.1 guidelines with:
- Proper ARIA labels and roles
- Keyboard navigation support
- Screen reader compatibility
- Sufficient color contrast

## Rate Limiting

The component implements client-side rate limiting (10 requests per minute) to prevent API abuse.

## Error Handling

- Network errors are displayed in the chat interface
- Graceful fallbacks when backend is unavailable
- Clear error messages for user guidance

## Performance

- Large text selections are limited to 2000 characters to prevent performance issues
- Efficient message history management
- Optimized rendering with React best practices